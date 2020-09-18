#include "ros_phoenix/falcon_component.hpp"

namespace ros_phoenix
{
    template <>
    void TalonFX::configure_current_limit(TalonFXConfiguration &config)
    {
        SupplyCurrentLimitConfiguration supply;
        supply.currentLimit = this->get_parameter("max_current").as_double();
        supply.triggerThresholdCurrent = this->get_parameter("max_current").as_double();
        supply.triggerThresholdTime = 100; //ms
        supply.enable = true;
        config.supplyCurrLimit = supply;
    }

    template <>
    void TalonFX::configure_sensor()
    {
        if (this->get_parameter("analog_input").as_bool())
            RCLCPP_WARN(this->get_logger(), "Analog input is not a valid option for a Falcon");
        this->controller_->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    }

    template <>
    double TalonFX::get_output_current()
    {
        return this->controller_->GetOutputCurrent();
    }
} // namespace ros_phoenix

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::TalonFX)