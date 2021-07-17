#ifndef ROS_PHOENIX_PHOENIX_COMPONENTS
#define ROS_PHOENIX_PHOENIX_COMPONENTS

#include "ros_phoenix/base_component.hpp"

namespace ros_phoenix
{

    typedef BaseComponent<ctre::phoenix::motorcontrol::can::TalonFX, TalonFXConfiguration, TalonFXFeedbackDevice, TalonFXControlMode> TalonFX;

    typedef BaseComponent<ctre::phoenix::motorcontrol::can::TalonSRX, TalonSRXConfiguration, TalonSRXFeedbackDevice, TalonSRXControlMode> TalonSRX;

    typedef BaseComponent<ctre::phoenix::motorcontrol::can::VictorSPX, VictorSPXConfiguration, FeedbackDevice, VictorSPXControlMode> VictorSPX;

}

#endif // ROS_PHOENIX_PHOENIX_COMPONENTS