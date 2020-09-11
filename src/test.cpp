#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/talon_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<ros_phoenix::TalonComponent>(rclcpp::NodeOptions());
    exec->add_node(node);
    exec->spin();
}