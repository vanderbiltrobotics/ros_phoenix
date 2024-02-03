/*

Author          Date                Description

K Nguyen        2-1-2024            removed BaseInterface as base class

                                    removed include header base_interface.hpp

                                    added rclcpp_lifecycle/state.hpp

                                    */

#ifndef ROS_PHOENIX_PHOENIX_SYSTEM
#define ROS_PHOENIX_PHOENIX_SYSTEM

#include <thread>

#include "ros_phoenix/base_node.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

// #include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"

namespace ros_phoenix {

class PhoenixSystem
    // : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
    : public hardware_interface::<hardware_interface::SystemInterface> {

public:
    static const std::string PERCENT_OUTPUT;
    static const std::string POSITION;
    static const std::string VELOCITY;

    RCLCPP_SHARED_PTR_DEFINITIONS(PhoenixSystem)

    PhoenixSystem();

    PhoenixSystem(PhoenixSystem&& other) = default;

    ~PhoenixSystem() = default;

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info);

    std::vector<hardware_interface::StateInterface> export_state_interfaces();

    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    hardware_interface::return_type start();

    hardware_interface::return_type stop();

    hardware_interface::return_type read();

    hardware_interface::return_type write();

private:
    struct JointInfo {
        hardware_interface::ComponentInfo info;
        BaseNode::SharedPtr node;
        ros_phoenix::msg::MotorControl::SharedPtr control;
        ros_phoenix::msg::MotorStatus::SharedPtr status;
    };

    static ControlMode str_to_interface(const std::string& str);

    rclcpp::Logger logger_;

    hardware_interface::HardwareInfo info_;

    std::vector<JointInfo> joints_;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
    std::thread spin_thread_;

    std::vector<rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr> subscribers_;
};

} // namespace ros_phoenix
#endif // ROS_PHOENIX_PHOENIX_SYSTEM
