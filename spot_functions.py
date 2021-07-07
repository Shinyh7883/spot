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
    val_list = [60, 60, 60, 60, 20, 20, 20, 20, 90, 90, 90, 90]

    def __init__(self):
        for i in range(len(self.val_list)):
            self.kit[0].servo[i].set_pulse_width_range(500,2500) #실험값
    
    def front_R(self, theta):
        num=[10, 6, 2]
        self.kit[0].servo[num[0]].angle=theta[0]
        self.kit[0].servo[num[1]].angle=(180 - theta[1])
        self.kit[0].servo[num[2]].angle=(180 - theta[2])

    def front_L(self, theta):
        num=[8, 4, 0]
        self.kit[0].servo[num[0]].angle=(180 - theta[0])
        self.kit[0].servo[num[1]].angle=theta[1]
        self.kit[0].servo[num[2]].angle=theta[2]

    def back_R(self, theta):
        num=[11, 7, 3]
        self.kit[0].servo[num[0]].angle=theta[0]
        self.kit[0].servo[num[1]].angle=(180 - theta[1])
        self.kit[0].servo[num[2]].angle=(180 - theta[2])

    def back_L(self, theta):
        num=[9, 5, 1]
        self.kit[0].servo[num[0]].angle=(180 - theta[0])
        self.kit[0].servo[num[1]].angle=theta[1]
        self.kit[0].servo[num[2]].angle=theta[2]


class kinematics:
    def leg_IK(self, location):
        ####길이, 각도####
        l1 = 50
        l2 = 105
        l3 = 120
        t1 = 80
        t2 = 0
        t3 = 10
        #################
        x = location[0]
        y = location[1]
        z = location[2]

        t1 = np.deg2rad(t1)
        t2 = np.deg2rad(t2)
        t3 = np.deg2rad(t3)

        # K = math.sqrt(y**2 + z**2)
        F = math.sqrt(y**2 + z**2 - l1**2)

        theta1 = np.deg2rad(180) - (math.atan2(F,l1) + math.atan2(y, z) + np.deg2rad(90) - t1)
        G = z - l1*math.sin(t1 - theta1)
        H = math.sqrt(x**2 + G**2)

        theta2 = math.acos((l2**2 + H**2 - l3**2)/(2* l2 * H)) + math.atan2(x, G) - t2
        theta3 = math.acos((l2**2 + l3**2 - H**2)/(2 * l2 * l3)) - t3
        theta = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)]
        print(theta)

        return theta