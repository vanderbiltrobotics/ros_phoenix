#ifndef ROS_PHOENIX_TALON_COMPONENT
#define ROS_PHOENIX_TALON_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<TalonSRX, TalonSRXConfiguration, TalonSRXFeedbackDevice, TalonSRXControlMode> TalonComponent;
} // namespace ros_phoenix

#endif // ROS_PHOENIX_TALON_COMPONENT
