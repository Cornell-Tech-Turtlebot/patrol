#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    print("0 is: %0.5f" % msg.ranges[0])
    print("90 is: %0.5f" % msg.ranges[90])
    print("180 is: %0.5f" % msg.ranges[180])
    print("270 is: %0.5f" % msg.ranges[270])


rospy.init_node('read_laser')
print("Initialized Node")
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()