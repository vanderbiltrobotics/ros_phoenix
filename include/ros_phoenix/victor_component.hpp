#ifndef ROS_PHOENIX_VICTOR_COMPONENT
#define ROS_PHOENIX_VICTOR_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<ctre::phoenix::motorcontrol::can::VictorSPX, VictorSPXConfiguration, FeedbackDevice, VictorSPXControlMode> VictorSPX;
} // namespace ros_phoenix

#endif // ROS_PHOENIX_VICTOR_COMPONENT
