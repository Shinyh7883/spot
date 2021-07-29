#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, String

def motion():
    rospy.init_node('ros_test1', anonymous=True)
    pub1 = rospy.Publisher('location', Int32MultiArray, queue_size=1) 
    pub2 = rospy.Publisher('leg', String, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        location_x = input("x좌표 입력하세요")
        location_y = input("y좌표 입력하세요")
        leg = input("다리명을 입력하세요(""front_R"")")
        a = String()
        a.data = leg
        b=Int32MultiArray()
        b.data=[int(location_x), int(location_y)]
        print(a)
        print(b)
        pub2.publish(a)
        pub1.publish(b)
        
        rate.sleep()
if __name__ == '__main__':
    motion()