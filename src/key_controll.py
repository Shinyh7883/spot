#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Int32MultiArray, String
def key_in():
    rospy.init_node('key_cont', anonymous=True)
    pub1 = rospy.Publisher('key', String, queue_size=1) 
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        key_input = input("press any key")
        a = String()
        a.data = key_input
        print(a)
        pub1.publish(a)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        
        rate.sleep()
if __name__ == '__main__':
    key_in()

    #중간에 멈추기 어떻게 함..