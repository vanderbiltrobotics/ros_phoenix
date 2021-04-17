#include <ros/node_handle.h>

#include "ros_phoenix/FakeTalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace ros_phoenix {
// TODO: Refactor this to wrap in custom messages and fewer publishers/subscribers
FakeTalonNode::FakeTalonNode(const ros::NodeHandle& parent, const std::string& name, int id, const TalonConfig& config, const double _pot_low, const double _pot_high)
    : nh(parent)
    , _name(name)
    , server(mutex, nh)
    , _config(config)
    , talon(id)
    , statusPub(nh.advertise<MotorStatus>("status", 1))
    , setSub(nh.subscribe("set", 1, &FakeTalonNode::set, this))
    , lastUpdate(ros::Time::now()) // watchdog - turn off talon if we haven't gotten an update in a while
    , _controlMode(ControlMode::PercentOutput)
    , _output(0.0)
    , disabled(false)
    , configured(false)
    , not_configured_warned(false)
    , pot_low(_pot_low)
    , pot_high(_pot_high)
    , _lastPosition(_pot_low)
    , lastPosUpdate(ros::Time::now())
{
    ROS_INFO("POT LOW: %d", pot_low);
    ROS_INFO("POT HIGH: %d", pot_high);
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    server.updateConfig(_config);
    server.setCallback(boost::bind(&FakeTalonNode::reconfigure, this, _1, _2));
    talon.NeutralOutput();
}

void FakeTalonNode::set(MotorControl output)
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    this->_controlMode = (ControlMode) output.mode;
    this->_output = output.value;
    this->lastUpdate = ros::Time::now();
}

void FakeTalonNode::reconfigure(const TalonConfig& config, uint32_t level)
{
    if (level == 0) {
        boost::recursive_mutex::scoped_lock scoped_lock(mutex);
        ROS_INFO("Reconfigure called on %s", _name.c_str());
        this->_config = config;
        this->configured = false;
    }
}

void FakeTalonNode::configure()
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

void FakeTalonNode::update()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    // Configure Talon if necessary
    if (!this->configured) {
        this->configure();
    }

    // Set Talon output
    if (this->_output == 0.0 or !this->configured) {
        talon.NeutralOutput();
    } else {
        talon.Set(this->_controlMode, this->_output);
    }

    MotorStatus status;
    status.temperature = 0;
    status.bus_voltage = 0;

    status.output_percent = this->_output;
    status.output_voltage = 0;
    status.output_current = 0;

    status.fwd_limit = false;
    status.rev_limit = false;

    double diff = pot_high - pot_low;
    ros::Time cur = ros::Time::now();
    double dTime = (cur - lastPosUpdate).toSec();
    status.position = _lastPosition + ((diff > 0 ? 1 : -1) * this->_output * dTime * 20);
    lastPosUpdate = cur;
    double trueHigh = (diff > 0 ? pot_high : pot_low);
    double trueLow = (diff > 0 ? pot_low : pot_high);
    if (status.position > trueHigh) {
        status.position = trueHigh;
    }

    if (status.position < trueLow) {
        status.position = trueLow;
    }
    
    status.velocity = (status.position - _lastPosition) / dTime;

    statusPub.publish(status);
    _lastPosition = status.position;
}

void FakeTalonNode::configureStatusPeriod()
{
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    // Sets status frame polling rate for methods like getTemperature()
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
