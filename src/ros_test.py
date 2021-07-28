#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, Int32

def motion():
    rospy.init_node('ros_test1', anonymous=True)
    pub1 = rospy.Publisher('location', Int32MultiArray, queue_size=1) 
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        location_x = input("x좌표 입력하세요")
        location_y = input("y좌표 입력하세요")
        b=Int32MultiArray()
        b.data=[location_x, location_y]
        pub1.publish(b)
        
        rate.sleep()
if __name__ == '__main__':
    motion()