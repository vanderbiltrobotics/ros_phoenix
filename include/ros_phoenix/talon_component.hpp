#ifndef ROS_PHOENIX_TALON_COMPONENT
#define ROS_PHOENIX_TALON_COMPONENT

#include "ros_phoenix/msg/talon_control.hpp"
#include "ros_phoenix/msg/talon_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"

namespace ros_phoenix
{

    class TalonComponent : public rclcpp::Node
    {
    public:
        explicit TalonComponent(const rclcpp::NodeOptions &options);

        ~TalonComponent();

        void set(ros_phoenix::msg::TalonControl::UniquePtr msg);

    protected:
        rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter> &params);

        void configure();

        void onTimer();

    private:
        std::shared_ptr<TalonSRX> talon_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<ros_phoenix::msg::TalonStatus>::SharedPtr pub_;

        rclcpp::Subscription<ros_phoenix::msg::TalonControl>::SharedPtr sub_;

        //rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr reset_service_;

        bool configured_ = false;
        std::mutex config_mutex_;
        std::shared_ptr<std::thread> config_thread_;
    };

} // namespace ros_phoenix

#endif // ROS_PHOENIX_TALON_COMPONENT
