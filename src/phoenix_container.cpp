#include "ros_phoenix/phoenix_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include <memory>
#include <string>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto phoenix_manager = ros_phoenix::PhoenixManager::getInstance(exec);

    exec->add_node(phoenix_manager);
    exec->spin();

    return 0;
}
