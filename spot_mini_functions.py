import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
kit = list()
kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))

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

        return theta


    def move(self, theta, max_len): #다리별 각도값 입력
        
        for i in range(max_len):
            theta1 = theta[0][i] #'front_R'
            theta2 = theta[1][i] #'front_L'
            theta3 = theta[2][i] #'back_R'
            theta4 = theta[3][i] #'back_L'

            # leg == 'front_R'
            print(theta1[0])
            num=[10, 6, 2]
            kit[0].servo[num[0]].angle=theta1[0]
            kit[0].servo[num[1]].angle=(180 - theta1[1])
            kit[0].servo[num[2]].angle=(180 - theta1[2])

            # leg == 'front_L'
            num=[8, 4, 0]
            kit[0].servo[num[0]].angle=(180 - theta2[0])
            kit[0].servo[num[1]].angle=theta2[1]
            kit[0].servo[num[2]].angle=theta2[2]

            # leg == 'back_R'
            num=[11, 7, 3]
            kit[0].servo[num[0]].angle=theta3[0]
            kit[0].servo[num[1]].angle=(180 - theta3[1])
            kit[0].servo[num[2]].angle=(180 - theta3[2])

            # leg == 'back_L'
            num=[9, 5, 1]
            kit[0].servo[num[0]].angle=(180 - theta4[0])
            kit[0].servo[num[1]].angle=theta4[1]
            kit[0].servo[num[2]].angle=theta4[2]

            time.sleep(0.0001)

    
class control(functions):
    def commend_set(self, commend):
        for i in range(4):
            commend[i][0] = commend[i][1] #이동 완료한 상태로 전환
        print("set")

    def commend(self, commend, leg, dot, type): #현 위치에서 이동할 점 입력
        if type == "linear":
            type = 1
        elif type == "direct":
            type = 2
        else:
            print("방법을 잘못 입력했어용")

        if leg == "front_R":
            commend[0][1] = dot
            commend[0][2] = type
        elif leg == "front_L":
            commend[1][1] = dot
            commend[1][2] = type
        elif leg == "back_R":
            commend[2][1] = dot
            commend[2][2] = type
        elif leg == "back_L":
            commend[3][1] = dot
            commend[3][2] = type
        else:
            print("다리를 잘못 입력했어용")

        return commend


    def commend_run(self, commend):
        dt = 100 # 1000은 가변
        if commend[0][2] == 1:
            theta1 = control.linear(self, commend[0])
        elif commend[0][2] == 2:
            theta1 = control.direct(self, commend[0], dt)
        if commend[1][2] == 1:
            theta2 = control.linear(self, commend[1])
        elif commend[1][2] == 2:
            theta2 = control.direct(self, commend[1], dt) 
        if commend[2][2] == 1:
            theta3 = control.linear(self, commend[2])
        elif commend[2][2] == 2:
            theta3 = control.direct(self, commend[2], dt)
        if commend[3][2] == 1:
            theta4 = control.linear(self, commend[3])
        elif commend[3][2] == 2:
            theta4 = control.direct(self, commend[3], dt)

        theta_run = np.empty((4,1))
        len_theta = (len(theta1), len(theta2), len(theta3), len(theta4))

        max_len = dt
        for num in len_theta:
            if (max_len is None or num > max_len):
                max_len = num
        print(max_len)
        print(len_theta)
        theta1 = control.array_len_equalization(self, theta1, max_len)
        theta2 = control.array_len_equalization(self, theta2, max_len)
        theta3 = control.array_len_equalization(self, theta3, max_len)
        theta4 = control.array_len_equalization(self, theta4, max_len)
        print("len_equalization")
        theta_run = [theta1, theta2, theta3, theta4]
        self.move(theta_run, max_len)
       

    def array_len_equalization(self, theta, max_len): # 제일 긴 행렬에 맞추어 마지막값 복사
        if (len(theta < max_len)):
            for i in range(max_len - len(theta)):
                theta = np.append(theta, [theta[len(theta) - 1]], axis = 0)
        print("theta 생성 완료")

        return theta



    def linear(self, commend):  #속도 입력 해서 거리를 나누어 구간 갯수 정하자
        dot1 = commend[0]
        dot2 = commend[1]
        dot_x = np.linspace(dot1[0], dot2[0], 100)
        dot_y = np.linspace(dot1[1], dot2[1], 100)
        dot_z = np.linspace(dot1[2], dot2[2], 100)

        theta = np.empty((1, 3))
        for i in range(np.shape(dot_x)[0]): #0 아님 1 이다
            theta = np.append(theta, [functions.leg_IK(self, [dot_x[i], dot_y[i], dot_z[i]])], axis = 0)
        theta = np.delete(theta, [0, 0], axis = 0)

        return theta

    def direct(self, commend, dt):
        theta1 = functions.leg_IK(self, commend[0])
        theta2 = functions.leg_IK(self, commend[1])

        theta_x = np.linspace(theta1[0], theta2[0], dt)
        theta_y = np.linspace(theta1[1], theta2[1], dt)
        theta_z = np.linspace(theta1[2], theta2[2], dt)

        theta = np.empty((1,3))
        for i in range(len(theta_x)):
            dtheta = np.array([[theta_x[i], theta_y[i], theta_z[i]]])
            theta = np.append(theta, dtheta, axis = 0)
        theta = np.delete(theta, [0,0], axis = 0)

        print(theta)

        return theta

class motion(functions):
    def foward(self): #한 다리 드는 알고리즘 부터 구상하도록 하자
        dot1 = [-90, 15, 90]
        dot2 = [-60, 15, 80]
        dot3 = [-95, 15, 100]
        dot4 = [-110, 15, 120]
        dot5 = [-40, 15, 90]

        commend = [[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2]]

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend)


        control.commend_set(self,commend)
        control.commend(self, commend, "front_R",  dot3, "direct")
        control.commend(self, commend, "back_L", dot2, "direct")
        control.commend(self, commend, "front_L",  dot3, "direct")
        control.commend(self, commend, "back_R", dot3, "direct")
        control.commend_run(self, commend)

        control.commend_set(self, commend)
        control.commend(self, commend, "front_R", dot4, "linear")
        control.commend(self, commend, "front_L",  dot3, "direct")
        control.commend(self, commend, "back_R", dot3, "direct")
        control.commend_run(self, commend)

        control.commend_set(self, commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot5, "linear")
        control.commend(self, commend, "back_R", dot5, "linear")
        control.commend(self, commend, "front_L", dot5, "linear")
        control.commend_run(self, commend)

        control.commend_set(self, commend)
        control.commend(self, commend, "front_R", [-90, 15, 80], "linear")
        control.commend(self, commend, "back_L", [-110, 15, 100], "linear")
        control.commend_run(self, commend)


        control.commend_set(self,commend)
        control.commend(self, commend, "front_L",  dot4, "direct")
        control.commend_run(self, commend)


        control.commend_set(self, commend)
        control.commend(self, commend, "back_R", dot4, "linear")
        control.commend(self, commend, "front_L", dot2, "direct")
        control.commend(self, commend, "back_L", dot3, "direct")
        control.commend_run(self, commend)

        control.commend_set(self, commend)
        control.commend(self, commend, "back_R", dot3, "direct")
        control.commend_run(self, commend)

        control.commend_set(self, commend)
        control.commend(self, commend, "back_R", dot5, "linear")
        control.commend_run(self, commend)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend)

    def backward(self):
        pass
    def left(self):
        pass
    def right(self):
        pass

        

