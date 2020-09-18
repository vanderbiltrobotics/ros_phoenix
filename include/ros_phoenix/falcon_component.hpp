#ifndef ROS_PHOENIX_FALCON_COMPONENT
#define ROS_PHOENIX_FALCON_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<ctre::phoenix::motorcontrol::can::TalonFX, TalonFXConfiguration, TalonFXFeedbackDevice, TalonFXControlMode> TalonFX;
}

#endif // ROS_PHOENIX_FALCON_COMPONENT
