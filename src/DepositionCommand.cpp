#include "ros/ros.h"
#include "ros_phoenix/MotorControl.h"
#include "std_msgs/Float64.h"

static double depositionMotorOutput = 0.0;

void cmdCallback(const std_msgs::Float64 &msg){
    depositionMotorOutput = msg.data;
    ROS_INFO("DepositionMotor=%f", depositionMotorOutput);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "deposition_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ros_phoenix/deposition_motor_vel", 1, cmdCallback);
    
    ros::Publisher depositionPublisher = nh.advertise<ros_phoenix::MotorControl>("/deposition/set", 1);
    
    ros::Rate loop_rate(50);
    while (ros::ok()){
        ros_phoenix::MotorControlPtr deposition(new ros_phoenix::MotorControl);
        deposition->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        deposition->value = depositionMotorOutput;
        depositionPublisher.publish(deposition);
      
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
