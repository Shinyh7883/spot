#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, Int32
from spot_mini_functions import *
motion = motion()
B = [15, 50, 150]

def location_move(A):
    rate = rospy.Rate(10)
    motion.test2([A.data[0], A.data[1]])

def listener():
    rospy.init_node('ros_test2', anonymous=True)
    rospy.Subscriber('location', Int32MultiArray, location_move)

    rospy.spin()

if __name__ == '__main__':
    listener()