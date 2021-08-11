#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, Int32

def motion():
    rospy.init_node('ros_test1', anonymous=True)
    pub1 = rospy.Publisher('location', Int32, queue_size=1) 
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        run= input("1: 이동 2: 멈춤")
        b=Int32()
        b.data=int(run)
        print(b)
        pub1.publish(b)
        
        rate.sleep()
if __name__ == '__main__':
    motion()