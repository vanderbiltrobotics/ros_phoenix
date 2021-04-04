#include "ros/ros.h"
#include "ros_phoenix/MotorControl.h"
#include "ros_phoenix/ArmVelocities.h"

static double shoulderOutput = 0.0;
static double elbowOutput = 0.0;
static double wristOutput = 0.0;

void cmdCallback(const ros_phoenix::ArmVelocities::ConstPtr& msg){
    shoulderOutput = msg->shoulder_velocity;
    elbowOutput = msg->elbow_velocity;
    wristOutput = msg->wrist_velocity;
    ROS_INFO("Shoulder=%f Elbow=%f Wrist=%f", shoulderOutput, elbowOutput, wristOutput);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ros_phoenix/arm_velocities", 1, cmdCallback);
    
    ros::Publisher shoulderPublisher = nh.advertise<ros_phoenix::MotorControl>("/shoulder/set", 1);
    ros::Publisher elbowPublisher = nh.advertise<ros_phoenix::MotorControl>("/elbow/set", 1);
    ros::Publisher wristPublisher = nh.advertise<ros_phoenix::MotorControl>("/wrist/set", 1);
    
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros_phoenix::MotorControlPtr shoulder(new ros_phoenix::MotorControl);
        shoulder->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        shoulder->value = shoulderOutput;
        shoulderPublisher.publish(shoulder);
        
        ros_phoenix::MotorControlPtr elbow(new ros_phoenix::MotorControl);
        elbow->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        elbow->value = elbowOutput;
        elbowPublisher.publish(elbow);
        
        ros_phoenix::MotorControlPtr wrist(new ros_phoenix::MotorControl);
        wrist->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        wrist->value = wristOutput;
        wristPublisher.publish(wrist);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
