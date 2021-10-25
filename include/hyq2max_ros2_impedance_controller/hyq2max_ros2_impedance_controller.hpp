#ifndef HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER__HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_HPP_
#define HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER__HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hyq2max_ros2_impedance_controller/visibility_control.h"
#include "hyq2max_interfaces/msg/joints_references.hpp"
#include "control_toolbox/pid.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"

namespace hyq2max_ros2_impedance_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using CmdType = hyq2max_interfaces::msg::JointsReferences;

    class ImpedanceControl : public controller_interface::ControllerInterface
    {
    public:

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        ImpedanceControl();

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        CallbackReturn on_init() override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        HYQ2MAX_ROS2_IMPEDANCE_CONTROLLER_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      
    private:

        std::vector<control_toolbox::Pid> pid_controller;

        double q[12];
        double qd[12];
        double effort[12];

        double commanded_effort[12];
        double q_e[12];
        double qd_e[12];

        double qr[12];
        double qdr[12];

        // this term is need to define the computeCommand method,
        // but is not used, as the integral term is 0
        uint64_t dt = 1;  

    protected:

        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::vector<double> Kp_gains;
        std::vector<double> Kd_gains;

        const std::vector<std::string> allowed_state_interface_types_ = {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT,
        };

        const std::vector<std::string> allowed_command_interface_types_ = {
        hardware_interface::HW_IF_EFFORT,
        };    
       
        template <typename T>
        using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

        InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> test_interface;

        // std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>> joint_command_interface_;
    
        rclcpp::Subscription<CmdType>::SharedPtr joints_reference_subscriber_ = nullptr;
    
    };
} //namespace hyq2max

#endif