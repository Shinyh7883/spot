import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time

class leg:
    i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
    kit = list()
    kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))
    val_list = [60, 60, 120, 120, 20, 20, 160, 160, 90, 90, 90, 90]
    
    def front_R(self, theta):
        num=[10, 6, 2]
        angle=theta
        for i in range(len(num)):
            self.kit[0].servo[num[i]].angle=180 - theta[i]


class kinematics:
    def leg_IK(self, location):
        ####길이, 각도####
        l1 = 50
        l2 = 105
        l3 = 130
        t1 = 0
        t2 = 70
        t3 = 50
        #################
        x = location[0]
        y = location[1]
        z = location[2]


        theta1 = math.atan2(z,y) - np.deg2rad(t1)
        theta2 = math.acos((l2**2 + (x**2 + y**2 + z**2 - l1**2) - l3**2)/(2* l2 *math.sqrt(x**2 + y**2 + z**2 - l1**2))) - np.deg2rad(t2)
        theta3 = math.acos((l2**2 + l3** - (x**2 + y**2 + z**2 - l1**2))/(2* l2 * l3)) - np.deg2rad(t3)
        theta = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)]

        return theta