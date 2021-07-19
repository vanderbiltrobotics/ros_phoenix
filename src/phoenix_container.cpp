#include "ros_phoenix/phoenix_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include <memory>
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);

    node->declare_parameter<std::string>("interface", "can0");
    node->declare_parameter<int>("period_ms", 50);
    node->declare_parameter<int>("watchdog_ms", 200);

    ros_phoenix::PhoenixManager::createInstance(
            node->get_parameter("interface").as_string(),
            node->get_parameter("period_ms").as_int(),
            node->get_parameter("watchdog_ms").as_int());

    exec->add_node(node);
    exec->spin();

    return 0;
}
