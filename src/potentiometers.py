#!/usr/bin/python
import rospy
from ros_phoenix.msg import MotorStatus
from std_msgs.msg import Float64

# LOW = retracted
# HIGH = out

shoulder_pub = None
elbow_pub = None
wrist_pub = None


def interpolate(x1, y1, x2, y2, x):
	assert x1 <= x <= x2 or x2 <= x <= x1
	m = ((y2 - y1) / (x2 - x1))
	b = y1 - (m * x1)
	
	return (m * x) + b


def shoulder_callback(status):
    global shoulder_pub
    POT_LOW = 294
    POT_HIGH = 13
    ANGLE_LOW = 2
    ANGLE_HIGH = 3.222

    shoulder_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))


def elbow_callback(status):
    global elbow_pub
    POT_LOW = 1023
    POT_HIGH = 747
    ANGLE_LOW = -0.663
    ANGLE_HIGH = 0.506

    elbow_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))


# angles from urdf
def wrist_callback(status):
    global wrist_pub
    POT_LOW = 659
    POT_HIGH = 258
    ANGLE_LOW = -0.89
    ANGLE_HIGH = 0.174

    wrist_pub.publish(interpolate(POT_LOW, ANGLE_LOW, POT_HIGH, ANGLE_HIGH, status.position))


def main():
    global shoulder_pub, elbow_pub, wrist_pub
    rospy.init_node("potentiometers")

    shoulder_pub = rospy.Publisher("/shoulder/angle", Float64, queue_size=10)
    elbow_pub = rospy.Publisher("/elbow/angle", Float64, queue_size=10)
    wrist_pub = rospy.Publisher("/wrist/angle", Float64, queue_size=10)

    rospy.Subscriber("/shoulder/status", MotorStatus, shoulder_callback)
    rospy.Subscriber("/elbow/status", MotorStatus, elbow_callback)
    rospy.Subscriber("/wrist/status", MotorStatus, wrist_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
