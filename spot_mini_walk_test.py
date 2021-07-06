from adafruit_servokit import ServoKit
import board
import busio
import time
from spot_functions import *

leg = leg()
kinematics = kinematics()

i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))

kit = list()
kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))


if __name__ == '__main__':

    theta = kinematics.leg_IK([-60, 26, 150])    
    leg.front_R(theta)
    leg.front_L(theta)
    leg.back_R(theta)
    leg.back_L(theta)

    time.sleep(1)

    while (1):
        
        theta = kinematics.leg_IK([-60, 26, 120])
        leg.back_L(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-90, 26, 120])
        leg.front_R(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-90, 26, 120])
        leg.back_L(theta)
        time.sleep(1)


        theta = kinematics.leg_IK([-60, 26, 120])
        leg.back_R(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-90, 26, 120])
        leg.front_L(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-90, 26, 120])
        leg.back_R(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-75, 26, 130])
        leg.back_R(theta)
        leg.back_L(theta)

        theta = kinematics.leg_IK([-75, 26, 150])
        leg.front_L(theta)
        leg.front_R(theta)
        time.sleep(1)

        theta = kinematics.leg_IK([-60, 26, 150])
        leg.back_R(theta)
        leg.front_L(theta)

        theta = kinematics.leg_IK([-60, 26, 150])
        leg.back_L(theta)
        leg.front_R(theta)
        time.sleep(1)




        