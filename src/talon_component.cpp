#include "ros_phoenix/talon_component.hpp"

namespace ros_phoenix
{
    template <>
    void TalonSRX::configure_current_limit(TalonSRXConfiguration &config)
    {
        config.continuousCurrentLimit = this->get_parameter("max_current").as_double();
        config.peakCurrentLimit = this->get_parameter("max_current").as_double();
        config.peakCurrentDuration = 100; // ms

        this->controller_->EnableCurrentLimit(true);
    }

    template <>
    void TalonSRX::configure_sensor()
    {
        if (this->get_parameter("analog_input").as_bool())
            this->controller_->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
        else
            this->controller_->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
    }

    template <>
    double TalonSRX::get_output_current()
    {
        return this->controller_->GetOutputCurrent();
    }
} // namespace ros_phoenix

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::TalonSRX)