#ifndef ROS_PHOENIX_TALON_COMPONENT
#define ROS_PHOENIX_TALON_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<ctre::phoenix::motorcontrol::can::TalonSRX, TalonSRXConfiguration, TalonSRXFeedbackDevice, TalonSRXControlMode> TalonSRX;
} // namespace ros_phoenix

#endif // ROS_PHOENIX_TALON_COMPONENT
