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

// #include "hardware_interface/loaned_command_interface.hpp"

namespace hyq2max_ros2_impedance_controller
{

  // using hardware_interface::LoanedCommandInterface;

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
      auto_declare<double>("joint_HAA_Kp", 100);
      auto_declare<double>("joint_HAA_Kd", 1);
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

    auto callback = [this](const std::shared_ptr<CmdType> msg) -> void {
      for(int index = 0; index < 12; ++index)
      {
        qr[index] = msg->q_r[index];
        qdr[index] = msg->qd_r[index];
      }
    };

    joints_reference_subscriber_= node_->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(), callback);

    RCLCPP_INFO(logger, "Joint reference substriber created");

    for (int i = 0; i < 12; i++){
      pid_controller[i].initPid(300, 0, 10, 10000, -10000);
      qdr[i] = 0;  
    }
    
    RCLCPP_INFO(logger, "Impedance controller gains update");

    qr[0] = -0.05;
    qr[1] = 0.65;
    qr[2] = -1.18;
    qr[3] = -0.05;
    qr[4] =  0.65;
    qr[5] = -1.18;
    qr[6] = -0.05;
    qr[7] = -0.65;
    qr[8] = 1.18;
    qr[9] = -0.05;
    qr[10] = -0.65;
    qr[11] = 1.18;

  RCLCPP_INFO(logger, "%f, %f, %f", qr[0], qr[1], qr[2]);

    return CallbackReturn::SUCCESS;
  }

  // Fill ordered_interfaces with references to the matching interfaces
  // in the same order as in joint_names
  template <typename T>
  bool get_ordered_interfaces(
    std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
    const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
  {
    for (const auto & joint_name : joint_names)
    {
      for (auto & command_interface : unordered_interfaces)
      {
        if (
          (command_interface.get_name() == joint_name) &&
          (command_interface.get_interface_name() == interface_type))
        {
          ordered_interfaces.emplace_back(std::ref(command_interface));
        }
      }
    }
    return joint_names.size() == ordered_interfaces.size();
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
      if (!get_ordered_interfaces(
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
      if (!get_ordered_interfaces(
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
      joint_command_interface_[0][index].get().get_value());
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

      // RCLCPP_INFO(logger,"%ld,%f, %f", index, q[index],qd[index]);

      // calcule the position and velocity errors
      q_e[index] = qr[index] - q[index];
      qd_e[index] = qdr[index] - q[index];

      commanded_effort[index] = pid_controller[index].computeCommand(q_e[index], qd_e[index], dt);
      // joint_command_interface_[0][index].get().set_value(1000);
      
      // effort[index] = joint_state_interface_[2][index].get().get_value(commanded_effort[index]);
    }
    // RCLCPP_INFO(logger,"%f", joint_state_interface_[0][0].get().get_value());

    return controller_interface::return_type::OK;
  }     
} //End namespace hyq2max_ros2_impedance_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    hyq2max_ros2_impedance_controller::ImpedanceControl,
    controller_interface::ControllerInterface)