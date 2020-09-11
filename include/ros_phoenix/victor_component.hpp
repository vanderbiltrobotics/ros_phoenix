#ifndef ROS_PHOENIX_VICTOR_COMPONENT
#define ROS_PHOENIX_VICTOR_COMPONENT

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{
    typedef BaseComponent<VictorSPX, VictorSPXConfiguration, FeedbackDevice, VictorSPXControlMode> VictorComponent;
} // namespace ros_phoenix

#endif // ROS_PHOENIX_VICTOR_COMPONENT
