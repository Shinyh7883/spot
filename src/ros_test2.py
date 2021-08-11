#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, Int32
from spot_mini_functions import *
motion = motion()
B = [15, 50, 150]

def location_move(A):
    rate = rospy.Rate(10)
    dot = [55, 50, 170]

    commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]
    print(A.data)

    motion.foward(commend, A.data)

def listener():
    rospy.init_node('ros_test2', anonymous=True)
    rospy.Subscriber('location', Int32, location_move)

    rospy.spin()

if __name__ == '__main__':
    listener()