#include "motor_control/TalonNode.h"
#include <ros/ros.h>

using namespace motor_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle nh;
    std::vector<std::unique_ptr<TalonNode>> talons;

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    XmlRpc::XmlRpcValue v;
    nh.getParam("talons", v);
    std::for_each(v.begin(), v.end(), [&nh, &talons](auto p) {
        const std::string name(p.first);
        XmlRpc::XmlRpcValue v(p.second);
        if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            TalonConfig config;
            dynamic_reconfigure::Server<TalonConfig>().getConfigDefault(config);
            if (v.hasMember("id")) {
                int id = v["id"];
                if (v.hasMember("inverted"))
                    config.inverted = (bool)v["inverted"];
                if (v.hasMember("peak_voltage"))
                    config.peak_voltage = (double)v["peak_voltage"];
                if (v.hasMember("pot"))
                    config.pot = (bool)v["pot"];
                if (v.hasMember("invert_sensor"))
                    config.invert_sensor = (bool)v["invert_sensor"];
                if (v.hasMember("P"))
                    config.P = (double)v["P"];
                if (v.hasMember("I"))
                    config.I = (double)v["I"];
                if (v.hasMember("D"))
                    config.D = (double)v["D"];
                if (v.hasMember("F"))
                    config.F = (double)v["F"];

                auto node = ros::NodeHandle(nh, name);
                talons.push_back(std::make_unique<TalonNode>(node, name, id, config));
                ROS_INFO("Created Talon with name '%s' and id '%d'", name.c_str(), id);
            } else {
                ROS_WARN("Failed to create Talon '%s' with missing ID number!", name.c_str());
            }
        } else {
            ROS_INFO("Unrecognized Talon XML member: %s", v.toXml().c_str());
        }
    });

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ctre::phoenix::unmanaged::FeedEnable(100);

        std::for_each(talons.begin(), talons.end(), [](std::unique_ptr<TalonNode>& talon) { talon->update(); });

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}