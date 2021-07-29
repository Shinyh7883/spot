#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, String
from spot_mini_functions import *
motion = motion()
B = [15, 50, 150]

def location_move(A):
    rate = rospy.Rate(10)
    motion.oneleg_test([A.data[0], A.data[1]], leg)

def leg_callback(B):
    global leg
    leg = B.data

def listener():
    rospy.init_node('ros_test2', anonymous=True)
    rospy.Subscriber('leg', String, leg_callback)
    rospy.Subscriber('location', Int32MultiArray, location_move)
    

    rospy.spin()

if __name__ == '__main__':
    listener()