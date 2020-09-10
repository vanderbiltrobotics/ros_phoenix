#include "ros_phoenix/talon_component.hpp"

#include "rcutils/logging_macros.h"

#include <chrono>
#include <rclcpp/qos.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace ros_phoenix {

TalonComponent::TalonComponent(const NodeOptions& options)
    : Node("talon", options)
{
    this->set_on_parameters_set_callback(std::bind(&TalonComponent::reconfigure, this, std::placeholders::_1));
    this->declare_parameter<int>("id", 0);
    this->declare_parameter<int>("period_ms", 20);
    this->declare_parameter<int>("follow", -1);
    this->declare_parameter<int>("edges_per_rot", 4096); // Encoder edges per rotation (4096 is for built-in encoder)
    this->declare_parameter<bool>("invert", false);
    this->declare_parameter<bool>("invert_sensor", false);
    this->declare_parameter<bool>("brake_mode", true);
    this->declare_parameter<bool>("analog_input", false);
    this->declare_parameter<double>("max_voltage", 12);
    this->declare_parameter<double>("max_current", 30);

    this->declare_parameter<double>("P", 0);
    this->declare_parameter<double>("I", 0);
    this->declare_parameter<double>("D", 0);
    this->declare_parameter<double>("F", 0);

    std::string name(this->get_name());

    this->pub_ = this->create_publisher<ros_phoenix::msg::TalonStatus>(name + "/status", 1);

    this->sub_ = this->create_subscription<ros_phoenix::msg::TalonControl>(name + "/set", 1,
        std::bind(&TalonComponent::set, this, std::placeholders::_1));
}

TalonComponent::~TalonComponent()
{
    std::lock_guard<std::mutex> guard(this->config_mutex_);
    if (this->config_thread_) {
        this->configured_ = true;
        guard.~lock_guard();
        this->config_thread_->join();
    }
}

rcl_interfaces::msg::SetParametersResult
TalonComponent::reconfigure(const std::vector<Parameter>& params)
{
    std::lock_guard<std::mutex> guard(this->config_mutex_);
    this->configured_ = false;

    for (auto param : params) {
        RCLCPP_INFO(this->get_logger(),
            "Parameter changed: %s=%s",
            param.get_name().c_str(),
            param.value_to_string().c_str());
        if (param.get_name() == "id") {
            this->talon_.release();
            std::unique_ptr<TalonSRX> new_talon(std::make_unique<TalonSRX>(param.as_int()));
            this->talon_.swap(new_talon);
        } else if (param.get_name() == "period_ms") {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(param.as_int()),
                std::bind(&TalonComponent::onTimer, this));
        }
    }

    if (!this->config_thread_) { // Check if thread already exists
        this->config_thread_ = std::make_shared<std::thread>(std::bind(&TalonComponent::configure, this));
        this->config_thread_->detach();
    }

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
}

void TalonComponent::configure()
{
    bool warned = false;
    while (!this->configured_) {
        std::lock_guard<std::mutex> guard(this->config_mutex_);
        if (this->talon_->GetFirmwareVersion() == -1) {
            if (!warned) {
                RCLCPP_WARN(this->get_logger(), "Talon has not been seen and can not be configured!");
                warned = true;
            }
        } else {
            SlotConfiguration slot;
            slot.kP = this->get_parameter("P").as_double();
            slot.kI = this->get_parameter("I").as_double();
            slot.kD = this->get_parameter("D").as_double();
            slot.kF = this->get_parameter("F").as_double();

            TalonSRXConfiguration config;
            config.slot0 = slot;
            config.voltageCompSaturation = this->get_parameter("max_voltage").as_double();
            config.continuousCurrentLimit = this->get_parameter("max_current").as_double();
            config.peakCurrentLimit = this->get_parameter("max_current").as_double();
            config.peakCurrentDuration = 100; // ms
            config.pulseWidthPeriod_EdgesPerRot = this->get_parameter("edges_per_rot").as_int();

            ErrorCode error = this->talon_->ConfigAllSettings(config);
            if (error != ErrorCode::OK) {
                if (!warned) {
                    RCLCPP_WARN(this->get_logger(), "Talon configuration failed!");
                    warned = true;
                }
            } else {

                if (this->get_parameter("brake_mode").as_bool())
                    this->talon_->SetNeutralMode(NeutralMode::Brake);
                else
                    this->talon_->SetNeutralMode(NeutralMode::Coast);

                if (this->get_parameter("analog_input").as_bool())
                    this->talon_->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
                else
                    this->talon_->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);

                this->talon_->EnableCurrentLimit(true);
                this->talon_->EnableVoltageCompensation(true);
                this->talon_->SetInverted(this->get_parameter("inverted").as_bool());
                this->talon_->SetSensorPhase(this->get_parameter("invert_sensor").as_bool());
                this->talon_->SelectProfileSlot(0, 0);

                int follow = this->get_parameter("follow").as_int();
                if (follow > 0)
                    this->talon_->Set(ControlMode::Follower, follow);

                RCLCPP_INFO(this->get_logger(), "Successfully configured Talon");
                this->configured_ = true;
                this->config_thread_.reset();
            }
        }
        if (!this->configured_) {
            guard.~lock_guard(); // Release lock before sleeping
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void TalonComponent::onTimer()
{
    if (!this->configured_)
        return;

    ros_phoenix::msg::TalonStatus status;

    status.temperature = this->talon_->GetTemperature();
    status.bus_voltage = this->talon_->GetBusVoltage();

    status.output_percent = this->talon_->GetMotorOutputPercent();
    status.output_voltage = this->talon_->GetMotorOutputVoltage();
    status.output_current = this->talon_->GetOutputCurrent();

    status.position = this->talon_->GetSelectedSensorPosition();
    status.velocity = this->talon_->GetSelectedSensorVelocity();

    this->pub_->publish(status);
}

void TalonComponent::set(ros_phoenix::msg::TalonControl::UniquePtr msg)
{
    if (this->configured_)
        this->talon_->Set(static_cast<TalonSRXControlMode>(msg->mode), msg->value);
}

} // namespace ros_phoenix

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::TalonComponent)