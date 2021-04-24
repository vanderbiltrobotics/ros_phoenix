#!/usr/bin/python
import rospy
from ros_phoenix.msg import MotorStatus
from std_msgs.msg import Float64

# LOW = retracted
# HIGH = out

shoulder_pub = None
elbow_pub = None
wrist_pub = None
shoulder_vel_pub = None
elbow_vel_pub = None
wrist_vel_pub = None


def interpolate(x1, y1, x2, y2, x):
    print(x1,x2,x)
    assert ((x1 - 15) <= x <= (x2 + 15)) or ((x2 - 15) <= x <= (x1 + 15))
    m = ((y2 - y1) / (x2 - x1))
    b = y1 - (m * x1)
    
    return (m * x) + b


def shoulder_callback(status):
    global shoulder_pub
    global shoulder_vel_pub
    POT_LOW = 502
    POT_HIGH = 182
    ANGLE_LOW = 2
    ANGLE_HIGH = 3.222

    shoulder_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))
    shoulder_vel_pub.publish(status.velocity * abs((ANGLE_HIGH - ANGLE_LOW) / (POT_HIGH - POT_LOW)))


def elbow_callback(status):
    global elbow_pub
    global elbow_vel_pub
    POT_LOW = -278
    POT_HIGH = 0
    ANGLE_LOW = -0.663
    ANGLE_HIGH = 0.506

    elbow_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))
    elbow_vel_pub.publish(status.velocity * abs((ANGLE_HIGH - ANGLE_LOW) / (POT_HIGH - POT_LOW)))


# angles from urdf
def wrist_callback(status):
    global wrist_pub
    global wrist_vel_pub
    POT_LOW = 680
    POT_HIGH = 275
    ANGLE_LOW = 0.174
    ANGLE_HIGH = -0.89

    wrist_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))
    wrist_vel_pub.publish(status.velocity * abs((ANGLE_HIGH - ANGLE_LOW) / (POT_HIGH - POT_LOW)))


def main():
    global shoulder_pub, elbow_pub, wrist_pub, shoulder_vel_pub, elbow_vel_pub, wrist_vel_pub
    rospy.init_node("potentiometers")

    shoulder_pub = rospy.Publisher("/shoulder/angle", Float64, queue_size=10)
    elbow_pub = rospy.Publisher("/elbow/angle", Float64, queue_size=10)
    wrist_pub = rospy.Publisher("/wrist/angle", Float64, queue_size=10)

    shoulder_vel_pub = rospy.Publisher("/shoulder/vel", Float64, queue_size=10)
    elbow_vel_pub = rospy.Publisher("/elbow/vel", Float64, queue_size=10)
    wrist_vel_pub = rospy.Publisher("/wrist/vel", Float64, queue_size=10)

    rospy.Subscriber("/shoulder/status", MotorStatus, shoulder_callback)
    rospy.Subscriber("/elbow/status", MotorStatus, elbow_callback)
    rospy.Subscriber("/wrist/status", MotorStatus, wrist_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
