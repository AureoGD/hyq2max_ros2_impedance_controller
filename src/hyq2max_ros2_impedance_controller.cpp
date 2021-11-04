#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

#include "hyq2max_ros2_impedance_controller/hyq2max_ros2_impedance_controller.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "angles/angles.h"

#include "controller_interface/helpers.hpp"

namespace hyq2max_ros2_impedance_controller
{

  ImpedanceControl::ImpedanceControl()
  : controller_interface::ControllerInterface(), joint_names_({})
  {
  }
   
  CallbackReturn ImpedanceControl::on_init()
  {
    try
    {
      auto_declare<std::vector<std::string>>("joints", joint_names_);
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

      auto_declare<double>("joint_impedance_controller_gains.HAA_Kp", 100.0);
      auto_declare<double>("joint_impedance_controller_gains.HAA_Kd", 10.0);
      auto_declare<double>("joint_impedance_controller_gains.HFE_Kp", 100.0);
      auto_declare<double>("joint_impedance_controller_gains.HFE_Kd", 10.0);
      auto_declare<double>("joint_impedance_controller_gains.KFE_Kp", 100.0);
      auto_declare<double>("joint_impedance_controller_gains.KFE_Kd", 10.0);
      auto_declare<std::vector<double>>("joints_references", {});

      auto_declare<double>("robot_states_feedabck_rate", 100.0);
    }
    catch (const std::exception & e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    pid_controller.resize(12);

    return CallbackReturn::SUCCESS;
  }
    
  controller_interface::InterfaceConfiguration
  ImpedanceControl::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto & joint : joint_names_)
    {
      for (const auto & interface_type : command_interface_types_)
      {
        command_interfaces_config.names.push_back(joint + "/" + interface_type);
      }
    }
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  ImpedanceControl::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint_name : joint_names_)
      {
        for (const auto & interface_type : state_interface_types_)
        {
          state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
        }
      }
    return state_interfaces_config;
  }   

  CallbackReturn ImpedanceControl::on_configure(
  const rclcpp_lifecycle::State &)
  {
    const auto logger = node_->get_logger();

    joint_names_ = node_->get_parameter("joints").as_string_array();

    if (joint_names_.empty())
    {
      RCLCPP_WARN(logger, "'joints' parameter is empty.");
    }

    // Command interface checking
    // Specialized, child controllers set interfaces before calling configure function.
    if (command_interface_types_.empty())
    {
      command_interface_types_ = node_->get_parameter("command_interfaces").as_string_array();
    }

    if (command_interface_types_.empty())
    {
      RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
      return CallbackReturn::FAILURE;
    }

    for (const auto & interface : command_interface_types_)
    {
      auto it =
      std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
      if (it == allowed_command_interface_types_.end())
      {
        RCLCPP_ERROR(logger, "Command interface type '%s' not allowed! Only effort type is allowed!", interface.c_str());
        return CallbackReturn::FAILURE;
      }
    }
    
    joint_command_interface_.resize(allowed_command_interface_types_.size());

    // State interface checking
    state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();

    if (state_interface_types_.empty())
    {
      RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
      return CallbackReturn::FAILURE;
    }

    joint_state_interface_.resize(allowed_state_interface_types_.size());

    // Pritig format
    auto get_interface_list = [](const std::vector<std::string> & interface_types) {
      std::stringstream ss_interfaces;
        for (size_t index = 0; index < interface_types.size(); ++index)
          {
            if (index != 0)
          {
            ss_interfaces << " ";
          }
            ss_interfaces << interface_types[index];
          }
          return ss_interfaces.str();
    };

    RCLCPP_INFO(
      logger, "Command interfaces are [%s] and and state interfaces are [%s].",
      get_interface_list(command_interface_types_).c_str(),
      get_interface_list(state_interface_types_).c_str());
    
    // Create the topic where the joints reference are published

    auto callback_refs = [this](const std::shared_ptr<CmdType> msg) -> void {
      for(int index = 0; index < 12; ++index)
      {
        qr[index] = msg->q_r[index];
        qdr[index] = msg->qd_r[index];
      }
    };
    
    joints_reference_subscriber_= node_->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(), callback_refs);

    RCLCPP_INFO(logger, "Joint reference subscriber created");

    auto callback_odom = [this](const std::shared_ptr<OdometryMsg> msg) -> void {
      b[0] = msg-> pose.pose.position.x;
      b[1] = msg-> pose.pose.position.y;
      b[2] = msg-> pose.pose.position.z;
      
      Q[0] = msg-> pose.pose.orientation.x;
      Q[1] = msg-> pose.pose.orientation.y;
      Q[2] = msg-> pose.pose.orientation.z;
      Q[3] = msg-> pose.pose.orientation.w;

      bd[0] = msg->twist.twist.linear.x;
      bd[1] = msg->twist.twist.linear.y;
      bd[2] = msg->twist.twist.linear.z;

      omega[0] = msg->twist.twist.angular.x;
      omega[1] = msg->twist.twist.angular.y;
      omega[2] = msg->twist.twist.angular.z;
    };

    odometry_subscriber_ = node_->create_subscription<OdometryMsg>(
    "/hyq2max/odom", rclcpp::SensorDataQoS(), callback_odom);

    // States publisher

    const double state_publish_rate = node_->get_parameter("robot_states_feedabck_rate").get_value<double>();
    
    RCLCPP_INFO(logger, "Robot states will be published at %.2f Hz.", state_publish_rate);
    if (state_publish_rate > 0.0)
    {
      state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);
    }
    else
    {
      state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
    }

    publisher_ = node_->create_publisher<RobotStatesMsg>("~/robot_states", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<StatePublisher>(publisher_);

    last_state_publish_time_ = node_->now();

    // Gains update

    Kp_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.HAA_Kp").get_value<double>());
    Kp_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.HFE_Kp").get_value<double>());
    Kp_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.KFE_Kp").get_value<double>());

    Kd_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.HAA_Kd").get_value<double>());
    Kd_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.HFE_Kd").get_value<double>());
    Kd_gains.push_back(node_->get_parameter("joint_impedance_controller_gains.KFE_Kd").get_value<double>());

    // Initial joits reference

    std::vector<double> joits_references = node_->get_parameter("joints_references").get_value<std::vector<double>>();

    // Configure the PD controntroller and joints references
    int k = 0;
    for (int i = 0; i<4; i++)
    {
      for(int j = 0; j<3; j++)
      {
        pid_controller[k].initPid(Kd_gains[j], 0, Kd_gains[j], 10000, -10000);
        qr[k] = joits_references[k];
        qdr[k] = 0;
        k++; 
      }
    }    
    
    RCLCPP_INFO(logger, "Impedance controller gains update");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ImpedanceControl::on_activate(const rclcpp_lifecycle::State &)
  {

    const auto logger = node_->get_logger();
    RCLCPP_INFO(logger,"Activing impedance controller");

    // order all joints in the storage
    for (const auto & interface : command_interface_types_)
    {
      auto it =
      std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
      auto index = std::distance(allowed_command_interface_types_.begin(), it);   
      if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
      {
        RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
        interface.c_str(), joint_command_interface_[index].size());
        return CallbackReturn::ERROR;
      }
    }
   
    for (const auto & interface : state_interface_types_)
    {
      auto it =
      std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
      auto index = std::distance(allowed_state_interface_types_.begin(), it);
      if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
        {
          RCLCPP_ERROR(
          node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
          interface.c_str(), joint_state_interface_[index].size());
          return CallbackReturn::ERROR;
        }
      }    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ImpedanceControl::on_deactivate(const rclcpp_lifecycle::State &)
  {
    const auto logger = node_->get_logger();
    RCLCPP_INFO(logger,"Deactiveting impedance controller");

    // TODO(anyone): How to halt when using effort commands?
    for (auto index = 0ul; index < joint_names_.size(); ++index)
    {
      joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[2][index].get().get_value());
    }

    for (auto index = 0ul; index < allowed_state_interface_types_.size(); ++index)
    {
      joint_command_interface_[index].clear();
      joint_state_interface_[index].clear();
    }
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type ImpedanceControl::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {   
    const auto logger = node_->get_logger();  

    // At joint_state_interface_ are all the joints interfaces. 
    // They separate by the name and type:
    //   0 - Position
    //   1 - Velocity
    //   2 - Effort

    // At joint_command_interface_ are all the joints commands. 
    // Only effort command is allowed (see allowed_command_interface_types_)

    for (auto index = 0ul; index < 12; ++index)
    {   
      // get the joint position
      q[index] = joint_state_interface_[0][index].get().get_value();

      // get the joint velocity
      qd[index] = joint_state_interface_[1][index].get().get_value(); 

      // calcule the position and velocity errors

      q_e[index] = qr[index] - q[index];
      qd_e[index] = qdr[index] - qd[index];

      // commanded_effort[index] = pid_controller[index].computeCommand(q_e[index], qd_e[index], dt);
      
      commanded_effort[index] = 300.0 * q_e[index] + 20.0 * qd_e[index];
      joint_command_interface_[0][index].get().set_value(commanded_effort[index]);
      
       effort[index] = joint_state_interface_[2][index].get().get_value();
    }
    publish_state();

    return controller_interface::return_type::OK;
  }

  void ImpedanceControl::publish_state()
  {
    if (state_publisher_period_.seconds() <= 0.0)
    {
      return;
    }

    if (node_->now() < (last_state_publish_time_ + state_publisher_period_))
    {
      return;
    }

    if (state_publisher_ && state_publisher_->trylock())
    {
      last_state_publish_time_ = node_->now();        

      for(int index = 0; index <= 11; index++)
      {
        state_publisher_->msg_.q[index] = q[index];
        state_publisher_->msg_.qd[index] = qd[index];
        state_publisher_->msg_.tau[index] = effort[index];
      }

      for(int index = 0; index <= 2; index++)
      {
        state_publisher_->msg_.b[index] = b[index];
        state_publisher_->msg_.bd[index] = bd[index];
        state_publisher_->msg_.omega[index] = omega[index];
      }  

       for(int index = 0; index <= 3; index++)
      {
        state_publisher_->msg_.orientation[index] = Q[index];
      } 


      state_publisher_->unlockAndPublish();
    }
  }

} //End namespace hyq2max_ros2_impedance_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    hyq2max_ros2_impedance_controller::ImpedanceControl,
    controller_interface::ControllerInterface)