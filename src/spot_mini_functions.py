# -*- coding: utf-8 -*-
import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time
from sympy import Symbol, solve

i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
kit = list()
kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))

for i in range(12):
    kit[0].servo[i].set_pulse_width_range(550,2600) #180도 돌음

dt = 0.001 #각 구간 이동하는데 걸리는 시간

class functions:
    def leg_IK(self, location):
        ####길이, 각도####
        l1 = 50
        l2 = 105
        l3 = 120
        t1 = 90
        t2 = -90
        t3 = 70
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
            num=[10, 6, 2]
            kit[0].servo[num[0]].angle=(180 - theta1[0]) - 7
            kit[0].servo[num[1]].angle=(180 - theta1[1]) + 5
            kit[0].servo[num[2]].angle=(180 - theta1[2]) - 10

            # leg == 'front_L'
            num=[8, 4, 0]
            kit[0].servo[num[0]].angle=theta2[0] + 3
            kit[0].servo[num[1]].angle=theta2[1]
            kit[0].servo[num[2]].angle=theta2[2]

            # leg == 'back_R'
            num=[11, 7, 3]
            kit[0].servo[num[0]].angle=(180 - theta3[0]) - 7
            kit[0].servo[num[1]].angle=(180 - theta3[1]) 
            kit[0].servo[num[2]].angle=(180 - theta3[2]) - 10
        
            # leg == 'back_L'
            num=[9, 5, 1]
            kit[0].servo[num[0]].angle=theta4[0] + 3
            kit[0].servo[num[1]].angle=theta4[1] -10
            kit[0].servo[num[2]].angle=theta4[2] + 20

            time.sleep(dt)

    def dot_move(self, axis, len, dott):
        axis_dic = {'x' : 0, 'y' : 1, 'z' : 2}
        axis = axis_dic[axis]
        dot_move = [dott[0], dott[1], dott[2]]
        dot_move[axis] = dot_move[axis] + len
        return dot_move

    def fifthpoly(self, dot1, dot2, vel):
        Dt = np.empty((1, 1))
        v = vel # 1초에 이동 거리

        D = math.sqrt((dot2[0] - dot1[0]) ** 2 + (dot2[1] - dot1[1]) ** 2 + (dot2[2] - dot1[2]) ** 2)
        T = 0
        t = D / v
        if t == 0:
            Dt = [D]
        else:
            while T <= t:
                Dt = np.append(Dt, [[(D * T ** 3.0 / (2.0 * t ** 3.0)) * (20.0 - 30.0 * T / t + 12.0 * T ** 2.0 / t ** 2.0)]], axis = 0)
                T += dt
            Dt = np.delete(Dt, [0, 0], axis = 0)

        
        return Dt

    def state_set(self, state):

        if state == 1:
            leg = ["front_R", "back_L"]
            state = 2

        elif state == 2:
            leg = ["front_L", "back_R"]
            state = 1
        
        return [[state, 0], leg]

class control(functions):
    def commend_set(self, commend):
        for i in range(4):
            commend[i][0] = commend[i][1] #이동 완료한 상태로 전환

        # print("set")

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
        # print(commend)
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
        # print("max_len :", max_len)
        theta1 = control.array_len_equalization(self, theta1, max_len)
        theta2 = control.array_len_equalization(self, theta2, max_len)
        theta3 = control.array_len_equalization(self, theta3, max_len)
        theta4 = control.array_len_equalization(self, theta4, max_len)
        # print("len_equalization")
        theta_run = [theta1, theta2, theta3, theta4]
        self.move(theta_run, max_len)

    def array_len_equalization(self, theta, max_len): # 제일 긴 행렬에 맞추어 마지막값 복사
        if (len(theta) < max_len):
            for i in range(max_len - len(theta)):
                theta = np.append(theta, [theta[len(theta) - 1]], axis = 0)

        return theta

    def linear(self, commend, vel):  #속도 입력 해서 거리를 나누어 구간 갯수 정하자
        dot1 = commend[0]
        dot2 = commend[1]

        D = math.sqrt((dot2[0] - dot1[0]) ** 2 + (dot2[1] - dot1[1]) ** 2 + (dot2[2] - dot1[2]) ** 2)
        if D == 0:
            theta = [functions.leg_IK(self, dot2)]
        else:
            Dt = functions.fifthpoly(self, dot1, dot2, vel)
            theta = np.empty((1, 3))
            for i in range(len(Dt)): #0 아님 1 이다
                dot = [ ((D - Dt[i]) * dot1[0] + Dt[i] * dot2[0]) / D, ((D - Dt[i]) * dot1[1] + Dt[i] * dot2[1]) / D, ((D - Dt[i]) * dot1[2] + Dt[i] * dot2[2]) / D]
                theta = np.append(theta, [functions.leg_IK(self, dot)], axis = 0)
            theta = np.delete(theta, [0, 0], axis = 0)
        
        return theta

    def direct(self, commend, vel): #vel_max = 350deg/sec
        theta1 = functions.leg_IK(self, commend[0])
        theta2 = functions.leg_IK(self, commend[1])

        [theta1_x, theta1_y, theta1_z] = theta1
        [theta2_x, theta2_y, theta2_z] = theta2

        dtheta = [abs(theta1_x - theta2_x), abs(theta1_y - theta2_y), abs(theta1_z - theta2_z)] #최대 각 변위 계산하여 구간 개수 정하기
        max_dtheta = dtheta[0]
        for num in dtheta:
            if (max_dtheta is None or num > max_dtheta):
                max_dtheta = num

        n = int(max_dtheta/(vel*dt)) + 1

        # print(n)

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

    dot = [55, 50, 170]

    #####치수#####
    L = 180 #길이
    W = 200 #너비
    #############

    def ready(self, B, state, commend, run):

        [state, leg] = functions.state_set(self, state)
        state = state[0]
        if B > 0:
            leg = [leg[1], leg[0]]
        

        leg_dic = {'front_R' : [-20, -20], 'front_L' : [-20, 20], 'back_R' : [20, -20], 'back_L' : [20, 20]}
        
        ###준비###
        motion.oneleg_move(self, leg_dic[leg[0]], B, leg[0], commend)
        motion.body_move(self, [-leg_dic[leg[0]][0], -leg_dic[leg[0]][1]], commend) 
        
        motion.oneleg_move(self, leg_dic[leg[1]], B, leg[1], commend)
        motion.body_move(self, [leg_dic[leg[1]][0], -leg_dic[leg[1]][1]], commend)

        return state
        
    def stop(self, B, state, commend, run):

        [state, leg] = functions.state_set(self, state)
        state = state[0]
        if B > 0:
            leg = [leg[1], leg[0]]

        leg_dic = {'front_R' : [-20, -20], 'front_L' : [-20, 20], 'back_R' : [20, -20], 'back_L' : [20, 20]}

        ###멈추기###
        motion.oneleg_move(self, leg_dic[leg[0]], B, leg[0], commend)
        motion.body_move(self, [-leg_dic[leg[0]][0], -leg_dic[leg[0]][1]], commend) 

        motion.oneleg_move(self, leg_dic[leg[1]], B, leg[1], commend)
        motion.body_move(self, [-leg_dic[leg[1]][0], -leg_dic[leg[1]][1]], commend)

        return state

    def fbchange_ready(self, B, state, commend, run):
        
        [state, leg] = functions.state_set(self, state)
        state = state[0]
        if B > 0:
            [state, leg] = functions.state_set(self, state)
            leg = [leg[1], leg[0]]
            state = state[0]

        leg_dic = {'front_R' : [-20, -20], 'front_L' : [-20, 20], 'back_R' : [20, -20], 'back_L' : [20, 20]}

        motion.body_move(self, [-2*leg_dic[leg[0]][0], 0], commend)

        return state

    def fbmove(self, B, state, commend, run):

        [state, leg] = functions.state_set(self, state)
        state = state[0]
        leg_dic = {'front_R' : [-20, -20], 'front_L' : [-20, 20], 'back_R' : [20, -20], 'back_L' : [20, 20]}

        if B > 0:
            [state, leg] = functions.state_set(self, state)
            leg = [leg[1], leg[0]]
            state = state[0]

        motion.oneleg_move(self, leg_dic[leg[0]], 2*B, leg[0], commend)
        motion.body_move(self, [-leg_dic[leg[0]][0], -leg_dic[leg[0]][1]], commend) 

        motion.oneleg_move(self, leg_dic[leg[1]], 2*B, leg[1], commend)
        motion.body_move(self, [leg_dic[leg[1]][0], -leg_dic[leg[1]][1]], commend)

        return state

    def foward_test(self, commend, run): 
        ###준비###
        a = self.cg_calc(commend, 5)[1]
        print(a)
        print([-a[0],-a[1]])
        motion.oneleg_move(self, [-a[0],-a[1]], -40, "front_R", commend)
        
        b = self.cg_calc(commend, 5)[4]
        a = [-a[0] + b[0], -a[1] + b[1]]

        print(a)
        print([-a[0],-a[1]])
        motion.oneleg_move(self, [-a[0],-a[1]], -40, "back_L", commend)

 
        ###걷기###
        while run == 1:
            b = self.cg_calc(commend, 5)[2]
            a = [-a[0] + b[0], -a[1] + b[1]]
            print(a)

            motion.oneleg_move(self, [-a[0],-a[1]], -80, "front_L", commend)

            b = self.cg_calc(commend, 5)[3]
            a = [-a[0] + b[0], -a[1] + b[1]]
            print(a)

            motion.oneleg_move(self, [-a[0],-a[1]], -80, "back_R", commend)


            b = self.cg_calc(commend, 5)[1]
            a = [-a[0] + b[0], -a[1] + b[1]]
            print(a)

            motion.oneleg_move(self, [-a[0],-a[1]], -80, "front_R", commend)

            
            b = self.cg_calc(commend, 5)[4]
            a = [-a[0] + b[0], -a[1] + b[1]]
            print(a)

            motion.oneleg_move(self, [-a[0],-a[1]], -80, "back_L", commend)



        ###멈추기###
        b = self.cg_calc(commend, 5)[2]
        a = [-a[0] + b[0], -a[1] + b[1]] 

        motion.oneleg_move(self, [-a[0],-a[1]], -40, "front_L", commend)

        b = self.cg_calc(commend, 5)[3]
        a = [-a[0] + b[0], -a[1] + b[1]] 

        motion.oneleg_move(self, [-a[0],-a[1]], -40, "back_R", commend)

    def backward(self, commend, run): 
        ###준비###
        
        motion.oneleg_move(self, [20, 20], 40, "back_L", commend)
        motion.body_move(self, [-20, -20], commend) 
        #print(commend)

        motion.oneleg_move(self, [-20, -20], 40, "front_R", commend)
        #print(commend)

        motion.body_move(self, [-20, 20], commend) 
        #print(commend)

        ###걷기###
        if run == 1:

            
            motion.oneleg_move(self, [20, -20], 80, "back_R", commend)
            motion.body_move(self, [-20, 20], commend) 

            motion.oneleg_move(self, [-20, 20], 80, "front_L", commend)

            motion.body_move(self, [-20, -20], commend)

            motion.oneleg_move(self, [20, 20], 80, "back_L", commend)
            motion.body_move(self, [-20, -20], commend) 

            motion.oneleg_move(self, [-20, -20], 80, "front_R", commend)

            motion.body_move(self, [-20, 20], commend)

        ###멈추기###
        
        motion.oneleg_move(self, [20, -20], 40, "back_R", commend)
        motion.body_move(self, [-20, 20], commend) 

        motion.oneleg_move(self, [-20, 20], 40, "front_L", commend)
        motion.body_move(self, [20, -20], commend) 

    def right(self, commend, run): #객체지향화 하기
        ###돌기###
        if run == 1:
            motion.body_rotate(self, 20, commend)

            motion.oneleg_rotate(self, [20, 25], "back_L", commend)

            motion.oneleg_rotate(self, [10, -15], "back_R", commend)
            
            motion.oneleg_rotate(self, [-30, 25], "front_L", commend) 

            motion.oneleg_rotate(self, [-20, -25], "front_R", commend)

    def left(self, commend, run): #객체지향화 하기
        ###돌기###
        if run == 1:
            motion.body_rotate(self, -20, commend)

            motion.oneleg_rotate(self, [20, -25], "back_R", commend)

            motion.oneleg_rotate(self, [10, 15], "back_L", commend)

            motion.oneleg_rotate(self, [-30, -25], "front_R", commend)

            motion.oneleg_rotate(self, [-20, 25], "front_L", commend) 

    def foward_right(self, B, commend, run): #객체지향화 하기
       if run == 1:
            motion.body_rotate(self, 20, commend)

            motion.oneleg_move_rotate(self, [20, 25], B, "back_L", commend)

            motion.oneleg_move_rotate(self, [10, -15], B, "back_R", commend)
            
            motion.oneleg_move_rotate(self, [-30, 25], B, "front_L", commend) 

            motion.oneleg_move_rotate(self, [-20, -25], B, "front_R", commend)
            
    def body_move(self, A, commend):

        walksp = 200
        front_R = functions.dot_move(self, 'x', A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', A[1], front_R)

        back_R = functions.dot_move(self, 'x', A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', A[1], back_R)

        front_L = functions.dot_move(self, 'x', A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', -A[1], front_L)

        back_L = functions.dot_move(self, 'x', A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', -A[1], back_L)

         
        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend)

    def body_rotate(self, t, commend):

        walksp = 200
        theta = abs(t)
        D = math.sqrt(self.W ** 2 + self.L ** 2) / 2
        a = D * math.sin(np.deg2rad(theta) + math.atan2(self.W, self.L)) - self.W/2
        b = self.L/2 - D * math.cos(np.deg2rad(theta) + math.atan2(self.W, self.L))
        if t <= 0:
            [a, b] = [-a, -b]

        # print([a, b])

        front_R = functions.dot_move(self, 'x', b, commend[0][0])
        front_R = functions.dot_move(self, 'y', -a, front_R)

        back_R = functions.dot_move(self, 'x', b, commend[2][0])
        back_R = functions.dot_move(self, 'y', a, back_R)

        front_L = functions.dot_move(self, 'x', -b, commend[1][0])
        front_L = functions.dot_move(self, 'y', a, front_L)

        back_L = functions.dot_move(self, 'x', -b, commend[3][0])
        back_L = functions.dot_move(self, 'y', -a, back_L)

         
        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend)

    def oneleg_move(self, A, B, leg, commend): #A 는 [20, 5] 가 적당

        walksp = 400
        front_R = functions.dot_move(self, 'x', A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', A[1], front_R)

        back_R = functions.dot_move(self, 'x', A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', A[1], back_R)

        front_L = functions.dot_move(self, 'x', A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', -A[1], front_L)

        back_L = functions.dot_move(self, 'x', A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', -A[1], back_L)

        
        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")

        control.commend_run(self, commend, 100)
        control.commend_set(self, commend) 

        leg_dic = {'front_R' : front_R, 'front_L' : front_L, 'back_R' : back_R, 'back_L' : back_L}

        leg_move = leg_dic[leg]

        dot3 = functions.dot_move(self, 'z', -20, leg_move)

        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend)

        dot3 = functions.dot_move(self, 'x', B, dot3)

        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend) 

        # time.sleep(0.1)

        
        dot3 = functions.dot_move(self, 'z', 20, dot3)

        control.commend(self, commend, leg, dot3, "linear")

        control.commend_run(self, commend, 100)
        control.commend_set(self, commend) 

        # time.sleep(0.1)

    def oneleg_rotate(self, A, leg, commend): #A 는 [20, 5] 가 적당

        walksp = 400
        front_R = functions.dot_move(self, 'x', A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', A[1], front_R)

        back_R = functions.dot_move(self, 'x', A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', A[1], back_R)

        front_L = functions.dot_move(self, 'x', A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', -A[1], front_L)

        back_L = functions.dot_move(self, 'x', A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', -A[1], back_L)

        
        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")

        control.commend_run(self, commend, 200)
        control.commend_set(self, commend) 

        leg_dic = {'front_R' : front_R, 'front_L' : front_L, 'back_R' : back_R, 'back_L' : back_L}
        move_dic = {'front_R' : [A[0], A[1]], 'front_L' : [A[0], -A[1]], 'back_R' : [A[0], A[1]], 'back_L' : [A[0], -A[1]]}
        leg_move = leg_dic[leg]

        dot3 = functions.dot_move(self, 'z', -20, leg_move)

        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend)

        dot3 = functions.dot_move(self, 'z', -20, self.dot)
        dot3 = functions.dot_move(self, 'x', move_dic[leg][0], dot3)
        dot3 = functions.dot_move(self, 'y', move_dic[leg][1], dot3)
        # print(dot3)


        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, 200)
        control.commend_set(self, commend) 

        # time.sleep(0.1)

        
        dot3 = functions.dot_move(self, 'z', 20, dot3)
        # print(dot3)

        control.commend(self, commend, leg, dot3, "linear")

        control.commend_run(self, commend, 100)
        control.commend_set(self, commend) 

        front_R = functions.dot_move(self, 'x', -A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', -A[1], front_R)

        back_R = functions.dot_move(self, 'x', -A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', -A[1], back_R)

        front_L = functions.dot_move(self, 'x', -A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', A[1], front_L)

        back_L = functions.dot_move(self, 'x', -A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', A[1], back_L)

        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")

        #print(commend)
        control.commend_run(self, commend, 200)
        control.commend_set(self, commend)

    def oneleg_move_rotate(self, A, B, leg, commend): #A 는 [20, 5] 가 적당

        walksp = 400
        front_R = functions.dot_move(self, 'x', A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', A[1], front_R)

        back_R = functions.dot_move(self, 'x', A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', A[1], back_R)

        front_L = functions.dot_move(self, 'x', A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', -A[1], front_L)

        back_L = functions.dot_move(self, 'x', A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', -A[1], back_L)

        
        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")

        control.commend_run(self, commend, 200)
        control.commend_set(self, commend) 

        leg_dic = {'front_R' : front_R, 'front_L' : front_L, 'back_R' : back_R, 'back_L' : back_L}
        move_dic = {'front_R' : [A[0], A[1]], 'front_L' : [A[0], -A[1]], 'back_R' : [A[0], A[1]], 'back_L' : [A[0], -A[1]]}
        leg_move = leg_dic[leg]

        dot3 = functions.dot_move(self, 'z', -20, leg_move)

        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, walksp)
        control.commend_set(self, commend)

        dot3 = functions.dot_move(self, 'z', -20, self.dot)
        dot3 = functions.dot_move(self, 'x', move_dic[leg][0], dot3)
        dot3 = functions.dot_move(self, 'x', B, dot3)
        dot3 = functions.dot_move(self, 'y', move_dic[leg][1], dot3)
        # print(dot3)


        control.commend(self, commend, leg, dot3, "linear")

        # print(commend)
        
        control.commend_run(self, commend, 200)
        control.commend_set(self, commend) 

        # time.sleep(0.1)

        
        dot3 = functions.dot_move(self, 'z', 20, dot3)
        # print(dot3)

        control.commend(self, commend, leg, dot3, "linear")

        control.commend_run(self, commend, 100)
        control.commend_set(self, commend) 

        front_R = functions.dot_move(self, 'x', -A[0], commend[0][0])
        front_R = functions.dot_move(self, 'y', -A[1], front_R)

        back_R = functions.dot_move(self, 'x', -A[0], commend[2][0])
        back_R = functions.dot_move(self, 'y', -A[1], back_R)

        front_L = functions.dot_move(self, 'x', -A[0], commend[1][0])
        front_L = functions.dot_move(self, 'y', A[1], front_L)

        back_L = functions.dot_move(self, 'x', -A[0], commend[3][0])
        back_L = functions.dot_move(self, 'y', A[1], back_L)

        control.commend(self, commend, "front_R", front_R, "linear")
        control.commend(self, commend, "back_R", back_R, "linear")
        control.commend(self, commend, "front_L", front_L, "linear")
        control.commend(self, commend, "back_L", back_L, "linear")

        #print(commend)
        control.commend_run(self, commend, 200)
        control.commend_set(self, commend)

    def cg_calc(self, commend, R): # R 은 내분비율
        dot = self.dot
   
        [l1, l2, w] = [40, 140, 75] #하드웨어 치수
        [dot1, dot2, dot3, dot4] = [commend[0][0], commend[1][0], commend[2][0], commend[3][0]]
        [fr, fl, br, bl] = [[-w/2-dot1[0], -l2+dot1[1]], [w/2+dot2[0], -l2+dot2[1]], [-w/2-dot3[0], l1+dot3[1]], [w/2+dot4[0], l1+dot4[1]]]

        a = float(((bl[1] - fr[1])/(bl[0] - fr[0])))
        b = float(fr[1] - a * fr[0])
        c = float(((fl[1] - br[1])/(fl[0] - br[0])))
        d = float(fl[1] - c * fl[0])

        x=Symbol('x') #교차점 구함
        equation = (a - c) * x + (b - d)

        c_x = float(solve(equation)[0])
        c_y = float(a * c_x + b)

        dot_c = [c_x, c_y]
        # print(dot_c)
        dot_fr = [(R * c_x + bl[0])/(R+1), (R * c_y + bl[1])/(R+1)]
        dot_fl = [(R * c_x + br[0])/(R+1), (R * c_y + br[1])/(R+1)]
        dot_br = [(R * c_x + fl[0])/(R+1), (R * c_y + fl[1])/(R+1)]
        dot_bl = [(R * c_x + fr[0])/(R+1), (R * c_y + fr[1])/(R+1)]

        return [dot_c, dot_fr, dot_fl, dot_br, dot_bl]

