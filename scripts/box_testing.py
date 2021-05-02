#!/usr/bin/env python

"""A script to control multiple talons as a constant current sink.
This script is designed for use with power resistors to place a
large known amount of load on the power box. It will automatically
balance the load across all of the talons and resistors. The
desired load is specified as a single command line argument of
the total system output current. It uses the talon's measured
bus voltages to dynamically adjust the output voltages to 
maintain a constant current output.
"""

import rospy
import sys
from ros_phoenix.msg import MotorControl, MotorStatus
import math

TALONS = ["talonA", "talonB", "talonC"]

PUBLISH_FREQUENCY   = 20  # Hz
OUTPUT_RESISTANCE   = 0.2 # Ohms
MAX_OUTPUT_VOLTAGE  = 12  # Volts
NOMINAL_BUS_VOLTAGE = 14.8 # Volts

pubs = {}
subs = {}
voltages = {talon: NOMINAL_BUS_VOLTAGE for talon in TALONS}

def status_handler(status, talon):
    if status.bus_voltage > MAX_OUTPUT_VOLTAGE:
        rospy.logdebug("%s: %.1f" % (talon, status.bus_voltage))
        voltages[talon] = status.bus_voltage

if __name__ == '__main__':
    rospy.init_node('box_testing', anonymous=True, log_level=rospy.DEBUG)

    current = rospy.get_param('~current', default=-1)
    if current == -1:
        rospy.logerr("Missing output current parameter!")
        exit()

    rospy.logdebug("Outputing a constant current of: %.1f amps" % current)
    rospy.logdebug("Spliting load across %d talons: %s" % (len(TALONS), ", ".join(TALONS)))

    # Create publishers and subscribers
    for talon in TALONS:
        pubs[talon] = rospy.Publisher('%s/set' % talon, MotorControl, queue_size=1)
        subs[talon] = rospy.Subscriber('%s/status' % talon, MotorStatus, status_handler, talon)

    rate = rospy.Rate(PUBLISH_FREQUENCY)
    while not rospy.is_shutdown():
        for talon in TALONS:
            # Calculate output voltage to achieve desired input current
            # v_in * i_in = v_out**2 / r_out
            v_in = voltages[talon]
            i_in = current / len(TALONS)
            v_out = math.sqrt(i_in * v_in * OUTPUT_RESISTANCE)

            # Limit v_out to voltage range
            v_out = min(v_out, MAX_OUTPUT_VOLTAGE)

            # Publish output percent
            msg = MotorControl()
            msg.mode = MotorControl.PERCENT_OUTPUT
            msg.value = v_out / MAX_OUTPUT_VOLTAGE
            pubs[talon].publish(msg)
            #rospy.logwarn("%s: %.1f %.1f %.1f" % (talon, v_in, v_out, msg.value))
        rate.sleep()