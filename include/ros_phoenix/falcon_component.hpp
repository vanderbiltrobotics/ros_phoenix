#ifndef ROS_PHOENIX_FALCON_COMPONENT
#define ROS_PHOENIX_FALCON_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<TalonFX, TalonFXConfiguration, TalonFXFeedbackDevice, TalonFXControlMode> FalconComponent;
}

#endif // ROS_PHOENIX_FALCON_COMPONENT
