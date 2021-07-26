import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
kit = list()
kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))
for i in range(12):
        kit[0].servo[i].set_pulse_width_range(600,2500) #180도 돌음

dt = 0.001 #각 구간 이동하는데 걸리는 시간

class functions:
    def leg_IK(self, location):
        ####길이, 각도####
        l1 = 50
        l2 = 105
        l3 = 120
        t1 = 80
        t2 = -90
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
            kit[0].servo[num[2]].angle=(180 - theta1[2]) + 30

            # leg == 'front_L'
            num=[8, 4, 0]
            kit[0].servo[num[0]].angle=(180 - theta2[0])
            kit[0].servo[num[1]].angle=theta2[1]
            kit[0].servo[num[2]].angle=theta2[2] -30

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

            time.sleep(dt)

    
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
            print("잘못 입력했어용")

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

        return commend


    def commend_run(self, commend, vel):
        if commend[0][2] == 1:
            theta1 = control.linear(self, commend[0], vel)
        elif commend[0][2] == 2:
            theta1 = control.direct(self, commend[0], vel)
        if commend[1][2] == 1:
            theta2 = control.linear(self, commend[1], vel)
        elif commend[1][2] == 2:
            theta2 = control.direct(self, commend[1], vel) 
        if commend[2][2] == 1:
            theta3 = control.linear(self, commend[2], vel)
        elif commend[2][2] == 2:
            theta3 = control.direct(self, commend[2], vel)
        if commend[3][2] == 1:
            theta4 = control.linear(self, commend[3], vel)
        elif commend[3][2] == 2:
            theta4 = control.direct(self, commend[3], vel)

        theta_run = np.empty((4,1))
        len_theta = (len(theta1), len(theta2), len(theta3), len(theta4))

        max_len = dt
        for num in len_theta:
            if (max_len is None or num > max_len):
                max_len = num
        print("max_len :", max_len)
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
        print(theta)

        return theta



    def linear(self, commend, vel):  #속도 입력 해서 거리를 나누어 구간 갯수 정하자
        dot1 = commend[0]
        dot2 = commend[1]

        [theta1_x, theta1_y, theta1_z] = functions.leg_IK(self, commend[0])
        [theta2_x, theta2_y, theta2_z] = functions.leg_IK(self, commend[1])

        print([theta1_x, theta1_y, theta1_z])

        dtheta = [abs(theta1_x - theta2_x), abs(theta1_y - theta2_y), abs(theta1_z - theta2_z)] #최대 각 변위 계산하여 구간 개수 정하기
        max_dtheta = dtheta[0]
        print(dtheta)
        for num in dtheta:
            if (max_dtheta is None or num > max_dtheta):
                max_dtheta = num

        print(max_dtheta)
        n = int(max_dtheta/(vel*dt)) + 1
        print(n)
        dot_x = np.linspace(dot1[0], dot2[0], n)
        dot_y = np.linspace(dot1[1], dot2[1], n)
        dot_z = np.linspace(dot1[2], dot2[2], n)

        theta = np.empty((1, 3))
        for i in range(np.shape(dot_x)[0]): #0 아님 1 이다
            theta = np.append(theta, [functions.leg_IK(self, [dot_x[i], dot_y[i], dot_z[i]])], axis = 0)
        theta = np.delete(theta, [0, 0], axis = 0)

        return theta

    def direct(self, commend, vel): #vel_max = 350deg/sec
        theta1 = functions.leg_IK(self, commend[0])
        theta2 = functions.leg_IK(self, commend[1])

        [theta1_x, theta1_y, theta1_z] = theta1
        [theta2_x, theta2_y, theta2_z] = theta2

        dtheta = [abs(theta1_x - theta2_x), abs(theta1_y - theta2_y), abs(theta1_z - theta2_z)] #최대 각 변위 계산하여 구간 개수 정하기
        print(dtheta)
        max_dtheta = dtheta[0]
        for num in dtheta:
            if (max_dtheta is None or num > max_dtheta):
                max_dtheta = num

        print(max_dtheta)
        n = int(max_dtheta/(vel*dt)) + 1

        print(n)

        theta_x = np.linspace(theta1[0], theta2[0], n)
        theta_y = np.linspace(theta1[1], theta2[1], n)
        theta_z = np.linspace(theta1[2], theta2[2], n)

        theta = np.empty((1,3))
        for i in range(len(theta_x)):
            dtheta = np.array([[theta_x[i], theta_y[i], theta_z[i]]])
            theta = np.append(theta, dtheta, axis = 0)
        theta = np.delete(theta, [0,0], axis = 0)
        
        return theta

class motion(functions):
    def foward(self): #캘리브레이션 -> 걷는 알고리즘 수작업으로 만들기
        dot1 = [0, 25, 120]
        dot2 = [30, 25, 100]
        dot3 = [0, 25, 130]
        dot4 = [-30, 25, 150]
        dot5 = [50, 25, 120]

        commend = [[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2]]

        walksp = 400

        control.commend_set(self,commend)
        control.commend(self, commend, "back_L", [20, 25, 90], "linear")
        control.commend(self, commend, "front_R", [20, 25, 130], "linear")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", [0, 25, 90], "direct")
        control.commend_run(self, commend, walksp)


        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", [-30, 25, 130], "linear")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend, 100)

        print('오른앞발 이동완료')

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", [20, 25, 90], "linear")
        control.commend(self, commend, "back_L", [20, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "back_L", [0, 25, 90], "linear")
        control.commend_run(self, commend, walksp)


        control.commend_set(self,commend)
        control.commend(self, commend, "back_L", [-30, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend, 100)

        print('왼뒷발 이동완료')

        control.commend_set(self,commend)
        control.commend(self, commend, "back_R", [20, 25, 90], "linear")
        control.commend(self, commend, "back_L", [20, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_L", [0, 25, 90], "linear")
        control.commend_run(self, commend, walksp)


        control.commend_set(self,commend)
        control.commend(self, commend, "front_L", [-30, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend, 100)

        print('왼앞발 이동완료')

        control.commend_set(self,commend)
        control.commend(self, commend, "front_L", [20, 25, 90], "linear")
        control.commend(self, commend, "back_R", [20, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "back_R", [0, 25, 90], "linear")
        control.commend_run(self, commend, walksp)


        control.commend_set(self,commend)
        control.commend(self, commend, "back_R", [-30, 25, 130], "direct")
        control.commend_run(self, commend, walksp)

        control.commend_set(self,commend)
        control.commend(self, commend, "front_R", dot1, "linear")
        control.commend(self, commend, "back_L", dot1, "linear")
        control.commend(self, commend, "back_R", dot1, "linear")
        control.commend(self, commend, "front_L", dot1, "linear")
        control.commend_run(self, commend, 100)

        print('오른뒷발 이동완료')

    def backward(self):
        pass
    def left(self):
        pass
    def right(self):
        pass

        

