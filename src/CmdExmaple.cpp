#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros_phoenix/MotorControl.h"

static double leftMotorOutput = 0.0;
static double rightMotorOutput = 0.0;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double moveValue = msg->linear.x;
    double rotateValue = msg->angular.z;
    if (moveValue > 0.0) {
        if (rotateValue > 0.0) {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = std::max(moveValue, rotateValue);
        } else {
            leftMotorOutput = std::max(moveValue, -rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        }
    } else {
        if (rotateValue > 0.0) {
            leftMotorOutput = -std::max(-moveValue, rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        } else {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = -std::max(-moveValue, -rotateValue);
        }
    }
    ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, cmdCallback);

    ros::Publisher fl = nh.advertise<ros_phoenix::MotorControl>("/front_left/set", 1);
    ros::Publisher fr = nh.advertise<ros_phoenix::MotorControl>("/front_right/set", 1);
    ros::Publisher bl = nh.advertise<ros_phoenix::MotorControl>("/back_left/set", 1);
    ros::Publisher br = nh.advertise<ros_phoenix::MotorControl>("/back_right/set", 1);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros_phoenix::MotorControlPtr left(new ros_phoenix::MotorControl);
        left->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        left->value = leftMotorOutput;
        fl.publish(left);
        bl.publish(left);

        ros_phoenix::MotorControlPtr right(new ros_phoenix::MotorControl);
        right->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        right->value = rightMotorOutput;
        fr.publish(right);
        br.publish(right);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
