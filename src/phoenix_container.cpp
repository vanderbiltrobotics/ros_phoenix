#include <memory>

#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_components/component_manager.hpp"

int main(int argc, char *argv[])
{
    // Stops the launch of the Phoenix diagnostic server so only one instance is started
    ctre::phoenix::platform::can::SetCANInterface("can0");
    c_SetPhoenixDiagnosticsStartTime(-1);

    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
    exec->add_node(node);
    exec->spin();
}
