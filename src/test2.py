from adafruit_servokit import ServoKit
import board
import busio
import time
from spot_functions import *
leg = leg()
kinematics = kinematics()
trajectory = trajectory()
dot1 = [0, 0, 150]
dot2 = [-60, 0, 150]
dot3 = [0, 0, 100]
while (1):
    dtheta = trajectory.steady(kinematics.leg_IK(dot1), kinematics.leg_IK(dot3), 300, 0.001, "front_L")
    dtheta_x = dtheta[0]
    dtheta_y = dtheta[1]
    dtheta_z = dtheta[2]
    leg_move = dtheta[3]
    print(dtheta_x[0])
    dn = 300
    dt = 0.001

    if leg_move == "front_L":
        for i_s in range(dn + 1):
            print(i_s)
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "front_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "back_L":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_L(theta)
            time.sleep(dt)

    elif leg_move == "back_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_R(theta)
            time.sleep(dt)


    dtheta = trajectory.steady(kinematics.leg_IK(dot3), kinematics.leg_IK(dot2), 300, 0.001, "front_L")
    dtheta_x = dtheta[0]
    dtheta_y = dtheta[1]
    dtheta_z = dtheta[2]
    leg_move = dtheta[3]
    print(dtheta_x[0])
    dn = 300
    dt = 0.001

    if leg_move == "front_L":
        for i_s in range(dn + 1):
            print(i_s)
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "front_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "back_L":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_L(theta)
            time.sleep(dt)

    elif leg_move == "back_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_R(theta)
            time.sleep(dt)

    dtheta = trajectory.steady(kinematics.leg_IK(dot2), kinematics.leg_IK(dot1), 300, 0.001, "front_L")
    dtheta_x = dtheta[0]
    dtheta_y = dtheta[1]
    dtheta_z = dtheta[2]
    leg_move = dtheta[3]
    print(dtheta_x[0])
    dn = 300
    dt = 0.001

    if leg_move == "front_L":
        for i_s in range(dn + 1):
            print(i_s)
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "front_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.front_L(theta)
            time.sleep(dt)

    elif leg_move == "back_L":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_L(theta)
            time.sleep(dt)

    elif leg_move == "back_R":
        for i_s in range(dn + 1):
            theta = [dtheta_x[i_s], dtheta_y[i_s], dtheta_z[i_s]]
            leg.back_R(theta)
            time.sleep(dt)