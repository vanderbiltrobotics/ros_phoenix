#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

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

    ros::Publisher fl = nh.advertise<std_msgs::Float64>("/front_left/set_percent_output", 1);
    ros::Publisher fr = nh.advertise<std_msgs::Float64>("/front_right/set_percent_output", 1);
    ros::Publisher bl = nh.advertise<std_msgs::Float64>("/back_left/set_percent_output", 1);
    ros::Publisher br = nh.advertise<std_msgs::Float64>("/back_right/set_percent_output", 1);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        std_msgs::Float64Ptr left(new std_msgs::Float64);
        left->data = leftMotorOutput;
        fl.publish(left);
        bl.publish(left);

        std_msgs::Float64Ptr right(new std_msgs::Float64);
        right->data = rightMotorOutput;
        fr.publish(right);
        br.publish(right);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
