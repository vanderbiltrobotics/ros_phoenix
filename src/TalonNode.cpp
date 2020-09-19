#include <ros/node_handle.h>

#include "ros_phoenix/TalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace ros_phoenix {

TalonNode::TalonNode(const ros::NodeHandle& parent, const std::string& name, int id, const TalonConfig& config)
    : nh(parent)
    , _name(name)
    , server(mutex, nh)
    , _config(config)
    , talon(id)
    , tempPub(nh.advertise<std_msgs::Float64>("temperature", 1))
    , busVoltagePub(nh.advertise<std_msgs::Float64>("bus_voltage", 1))
    , outputPercentPub(nh.advertise<std_msgs::Float64>("output_percent", 1))
    , outputVoltagePub(nh.advertise<std_msgs::Float64>("output_voltage", 1))
    , outputCurrentPub(nh.advertise<std_msgs::Float64>("output_current", 1))
    , analogPub(nh.advertise<std_msgs::Float64>("analog_input", 1))
    , posPub(nh.advertise<std_msgs::Int32>("position", 1))
    , velPub(nh.advertise<std_msgs::Int32>("velocity", 1))
    , fwdPub(nh.advertise<std_msgs::Bool>("forward_limit", 1))
    , revPub(nh.advertise<std_msgs::Bool>("reverse_limit", 1))
    , setPercentSub(nh.subscribe("set_percent_output", 1, &TalonNode::setPercentOutput, this))
    , setVelSub(nh.subscribe("set_velocity", 1, &TalonNode::setVelocity, this))
    , setPosSub(nh.subscribe("set_position", 1, &TalonNode::setPosition, this))
    , setCurSub(nh.subscribe("set_current", 1, &TalonNode::setCurrent, this))
    , lastUpdate(ros::Time::now())
    , _controlMode(ControlMode::PercentOutput)
    , _output(0.0)
    , disabled(false)
    , configured(false)
    , not_configured_warned(false)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    server.updateConfig(_config);
    server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
    talon.NeutralOutput();
}

void TalonNode::setPercentOutput(std_msgs::Float64 output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = ControlMode::PercentOutput;
    this->_output = output.data;
    this->lastUpdate = ros::Time::now();
}

void TalonNode::setVelocity(std_msgs::Float64 output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = ControlMode::Velocity;
    this->_output = output.data;
    this->lastUpdate = ros::Time::now();
}

void TalonNode::setPosition(std_msgs::Float64 output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = ControlMode::Position;
    this->_output = output.data;
    this->lastUpdate = ros::Time::now();
}

void TalonNode::setCurrent(std_msgs::Float64 output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = ControlMode::Position;
    this->_output = output.data;
    this->lastUpdate = ros::Time::now();
}

void TalonNode::reconfigure(const TalonConfig& config, uint32_t level)
{
    if (level == 0) {
        boost::recursive_mutex::scoped_lock scoped_lock(mutex);
        ROS_INFO("Reconfigure called on %s", _name.c_str());
        this->_config = config;
        this->configured = false;
    }
}

void TalonNode::configure()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    if (talon.GetFirmwareVersion() == -1) {
        if (!not_configured_warned) {
            ROS_WARN("Talon hasn't been seen: %d", talon.GetDeviceID());
            not_configured_warned = true;
        }
        return;
    }

    TalonSRXConfiguration c;
    SlotConfiguration slot;
    slot.kP = _config.P;
    slot.kI = _config.I;
    slot.kD = _config.D;
    slot.kF = _config.F;
    c.slot0 = slot;
    c.voltageCompSaturation = _config.peak_voltage;
    c.continuousCurrentLimit = _config.cont_current;
    c.peakCurrentLimit = _config.cont_current;
    c.peakCurrentDuration = _config.peak_current_dur;
    c.pulseWidthPeriod_EdgesPerRot = 4096;
    ErrorCode error = talon.ConfigAllSettings(c, 10);

    if (error != ErrorCode::OK) {
        if (!not_configured_warned) {
            ROS_WARN("Reconfiguring Talon %s %d failed!", _name.c_str(), talon.GetDeviceID());
            not_configured_warned = true;
        }
        this->configured = false;
    } else {
        configureStatusPeriod();

        if (_config.pot) {
            talon.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
        } else {
            talon.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
        }

        talon.EnableCurrentLimit(true);
        talon.SetSensorPhase(_config.invert_sensor);
        talon.SelectProfileSlot(0, 0);
        talon.SetInverted(_config.inverted);
        talon.EnableVoltageCompensation(true);

        if (_config.brake_mode) {
            talon.SetNeutralMode(NeutralMode::Brake);
        } else {
            talon.SetNeutralMode(NeutralMode::Coast);
        }

        ROS_INFO("Reconfigured Talon: %s with %d %f %f %f", _name.c_str(), talon.GetDeviceID(), _config.P, _config.I,
            _config.D);
        this->configured = true;
        this->not_configured_warned = false;
    }
}

void TalonNode::update()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    // Configure Talon if necessary
    if (!this->configured) {
        this->configure();
    }

    // Disable the Talon if we aren't getting commands
    if (ros::Time::now() - lastUpdate > ros::Duration(0.2)) {
        if (!this->disabled)
            ROS_WARN("Talon disabled for not receiving updates: %s", _name.c_str());
        this->disabled = true;
        this->_controlMode = ControlMode::PercentOutput;
        this->_output = 0.0;
    } else {
        if (this->disabled)
            ROS_INFO("Talon re-enabled for receiving updates: %s", _name.c_str());
        this->disabled = false;
    }

    // Set Talon output
    if (this->_output == 0.0 or !this->configured) {
        talon.NeutralOutput();
    } else {
        talon.Set(this->_controlMode, this->_output);
    }

    std_msgs::Float64 temperature;
    temperature.data = talon.GetTemperature();
    tempPub.publish(temperature);

    std_msgs::Float64 busVoltage;
    busVoltage.data = talon.GetBusVoltage();
    busVoltagePub.publish(busVoltage);

    std_msgs::Float64 outputPercent;
    outputPercent.data = talon.GetMotorOutputPercent();
    outputPercentPub.publish(outputPercent);

    std_msgs::Float64 outputVoltage;
    outputVoltage.data = talon.GetMotorOutputVoltage();
    outputVoltagePub.publish(outputVoltage);

    std_msgs::Float64 outputCurrent;
    outputCurrent.data = talon.GetOutputCurrent();
    outputCurrentPub.publish(outputCurrent);

    std_msgs::Float64 analogInput;
    analogInput.data = talon.GetSensorCollection().GetAnalogIn();
    analogPub.publish(analogInput);

    std_msgs::Bool forward_limit;
    forward_limit.data = talon.GetSensorCollection().IsFwdLimitSwitchClosed();
    fwdPub.publish(forward_limit);

    std_msgs::Bool reverse_limit;
    reverse_limit.data = talon.GetSensorCollection().IsRevLimitSwitchClosed();
    fwdPub.publish(reverse_limit);

    std_msgs::Int32 position;
    position.data = talon.GetSelectedSensorPosition();
    posPub.publish(position);

    std_msgs::Int32 velocity;
    velocity.data = talon.GetSelectedSensorVelocity() * 10;
    velPub.publish(velocity);
}

void TalonNode::configureStatusPeriod()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 100);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 100);
}

} // namespace ros_phoenix
