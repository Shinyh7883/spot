import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time

class functions:
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

        F = math.sqrt(y**2 + z**2 - l1**2)

        theta1 = np.deg2rad(180) - (math.atan2(F,l1) + math.atan2(y, z) + np.deg2rad(90) - t1)
        G = z - l1*math.sin(t1 - theta1)
        H = math.sqrt(x**2 + G**2)

        theta2 = math.acos((l2**2 + H**2 - l3**2)/(2* l2 * H)) + math.atan2(x, G) - t2
        theta3 = math.acos((l2**2 + l3**2 - H**2)/(2 * l2 * l3)) - t3
        theta = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)]
        print(theta)

        return theta

    def trajectory(self, dot1, dot2):
        pass

    def move(self, leg, theta):
        i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        kit = list()
        kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))

        if leg == 'front_R':
            num=[10, 6, 2]
            self.kit[0].servo[num[0]].angle=theta[0]
            self.kit[0].servo[num[1]].angle=(180 - theta[1])
            self.kit[0].servo[num[2]].angle=(180 - theta[2])

        if leg == 'front_L':
            num=[8, 4, 0]
            self.kit[0].servo[num[0]].angle=(180 - theta[0])
            self.kit[0].servo[num[1]].angle=theta[1]
            self.kit[0].servo[num[2]].angle=theta[2]

        if leg == 'back_R':
            num=[11, 7, 3]
            self.kit[0].servo[num[0]].angle=theta[0]
            self.kit[0].servo[num[1]].angle=(180 - theta[1])
            self.kit[0].servo[num[2]].angle=(180 - theta[2])

        if leg == 'back_L':
            num=[9, 5, 1]
            self.kit[0].servo[num[0]].angle=(180 - theta[0])
            self.kit[0].servo[num[1]].angle=theta[1]
            self.kit[0].servo[num[2]].angle=theta[2]

        else:
            pass

class control():
    def commend(self, commend, leg, dot1, dot2, type):
        if leg == "front_R":
            leg = 1
        elif leg == "front_L":
            leg = 2
        elif leg == "back_R":
            leg = 3
        elif leg == "back_L":
            leg = 4
        else:
            print("다리를 잘못 입력했어용")

        if type == "linear":
            type = 1
        elif type == "direct":
            type = 2
        else:
            print("방법을 잘못 입력했어용")

        # commend = np.zeros(1,3) # 실행파일 앞부분에 넣기!!
        commend = np.append(commend, [leg, dot1, dot2, type], axis = 0)
        print(commend)
        return commend

        # commend= np.delete(commend, [0, 0], axis = 0) #실행파일 뒷부분에 넣기!!

    def commend_perform(self, commend):
        len_commend = np.shape(commend)[0]
        if len_commend == 1:
            leg1 = commend[0][0]
            if len_commend == 2:
                leg2 = commend[1][0]
                if len_commend == 3:
                    leg3 = commend[2][0]
                    if len_commend == 4:
                        leg4 = commend[3][0]
                else:
                    pass #이거 수정해야함



        

    def linear(self, commend):
        dot1 = commend[1]
        dot2 = commend[2]
        dots = functions.trajectory(dot1, dot2)

        theta = np.zeros[1, 3]
        for i in range(np.shape(dots)[0]): #0 아님 1 이다
            theta = np.append(theta, functions.leg_IK(dots[i]), axis = 0)
        theta = np.delete(theta, [0, 0], axis = 0)

        return theta

    def direct(self, commend, dt):
        theta1 = functions.leg_IK(self, commend[1])
        theta2 = functions.leg_IK(self, commend[2])

        theta_x = np.linspace(theta1[0], theta2[0], dt)
        theta_y = np.linspace(theta1[1], theta2[1], dt)
        theta_z = np.linspace(theta1[2], theta2[2], dt)

        theta = np.zeros[1, 3]
        for i in range(len(theta_x)):
            theta = np.append(theta, [theta_x, theta_y, theta_z], axis = 0)
        theta = np.delete(theta, [0,0], axis = 0)

        return theta

class motion:
    def foward(self):
        pass
    def backward(self):
        pass
    def left(self):
        pass
    def right(self):
        pass

        

