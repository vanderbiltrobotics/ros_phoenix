#ifndef ROS_PHOENIX_BASE_COMPONENT
#define ROS_PHOENIX_BASE_COMPONENT

#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"

namespace ros_phoenix
{
    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    class BaseComponent : public rclcpp::Node
    {
    public:
        explicit BaseComponent(const rclcpp::NodeOptions &options);

        ~BaseComponent();

        void set(ros_phoenix::msg::MotorControl::UniquePtr msg);

    protected:
        rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter> &params);

        void configure();

        void configure_current_limit(Configuration &config);

        void configure_sensor();

        double get_output_current();

        void onTimer();

    private:
        int period_ms_;
        int watchdog_ms_;
        bool watchdog_warned_ = false;

        std::shared_ptr<MotorController> controller_;

        rclcpp::Time last_update_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr pub_;

        rclcpp::Subscription<ros_phoenix::msg::MotorControl>::SharedPtr sub_;

        //rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr reset_service_;

        bool configured_ = false;
        std::mutex config_mutex_;
        std::shared_ptr<std::thread> config_thread_;
    };

} // namespace ros_phoenix

#include "ros_phoenix/base_component.cpp"

#endif // ROS_PHOENIX_BASE_COMPONENT
