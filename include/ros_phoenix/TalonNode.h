#ifndef ROS_PHOENIX_TALONNODE_H
#define ROS_PHOENIX_TALONNODE_H

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "ros_phoenix/TalonConfig.h"
#include "ros_phoenix/MotorControl.h"
#include "ros_phoenix/MotorStatus.h"

#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace ros_phoenix {
class TalonNode {
private:
    boost::recursive_mutex mutex;
    ros::NodeHandle nh;
    std::string _name;
    dynamic_reconfigure::Server<TalonConfig> server;
    TalonConfig _config;

    TalonSRX talon;

    ros::Publisher statusPub;

    ros::Subscriber setSub;

    ros::Time lastUpdate;
    ControlMode _controlMode;
    double _output;
    bool disabled;
    bool configured;
    bool not_configured_warned;

public:
    TalonNode(const ros::NodeHandle& parent, const std::string& name, int id, const TalonConfig& config);

    TalonNode& operator=(const TalonNode&) = delete;

    ~TalonNode() = default;

    void reconfigure(const TalonConfig& config, uint32_t level);

    void configure();

    void set(MotorControl output);

    void update();

    void configureStatusPeriod();
};

} // namespace ros_phoenix

#endif // ROS_PHOENIX_TALONNODE_H
