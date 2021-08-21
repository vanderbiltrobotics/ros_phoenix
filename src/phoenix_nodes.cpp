#include "ros_phoenix/phoenix_nodes.hpp"

namespace ros_phoenix {

TalonFXNode::TalonFXNode(const std::string& name, const NodeOptions& options)
    : PhoenixNode(name, options)
{
}

void TalonFXNode::configure_current_limit(TalonFXConfiguration& config)
{
    SupplyCurrentLimitConfiguration supply;
    supply.currentLimit = this->get_parameter("max_current").as_double();
    supply.triggerThresholdCurrent = this->get_parameter("max_current").as_double();
    supply.triggerThresholdTime = 100; // ms
    supply.enable = true;
    config.supplyCurrLimit = supply;
}

void TalonFXNode::configure_sensor()
{
    if (this->get_parameter("analog_input").as_bool())
        RCLCPP_WARN(this->get_logger(), "Analog input is not a valid option for a Falcon");
    this->controller_->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
}

double TalonFXNode::get_output_current() { return this->controller_->GetOutputCurrent(); }

TalonSRXNode::TalonSRXNode(const std::string& name, const NodeOptions& options)
    : PhoenixNode(name, options)
{
}

void TalonSRXNode::configure_current_limit(TalonSRXConfiguration& config)
{
    config.continuousCurrentLimit = this->get_parameter("max_current").as_double();
    config.peakCurrentLimit = this->get_parameter("max_current").as_double();
    config.peakCurrentDuration = 100; // ms

    this->controller_->EnableCurrentLimit(true);
}

void TalonSRXNode::configure_sensor()
{
    if (this->get_parameter("analog_input").as_bool())
        this->controller_->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
    else
        this->controller_->ConfigSelectedFeedbackSensor(
            TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
}

double TalonSRXNode::get_output_current() { return this->controller_->GetOutputCurrent(); }

VictorSPXNode::VictorSPXNode(const std::string& name, const NodeOptions& options)
    : PhoenixNode(name, options)
{
}

void VictorSPXNode::configure_current_limit(VictorSPXConfiguration& config __attribute__((unused)))
{
}
void VictorSPXNode::configure_sensor() { }

double VictorSPXNode::get_output_current() { return 0.0; }

} // namespace ros_phoenix