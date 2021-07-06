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

    stand = kinematics.leg_IK([-90, 20, 100]) 
    up_in = kinematics.leg_IK([-110, 0, 110])
    up_out = kinematics.leg_IK([-110, 25, 110])  
    down = kinematics.leg_IK([-100, 20, 90])
    walk = kinematics.leg_IK([-100, 20, 80]) 

    while (1):
        leg.back_L(down)
        time.sleep(0.3)
        leg.back_R(up_out)
        time.sleep(0.3)
        leg.front_L(up_in)
        time.sleep(0.3)

        leg.front_R(up_out)
        time.sleep(0.5)

        leg.front_R(walk)
        time.sleep(2) #오른 앞발
        print("오른 앞발")

        # leg.front_R(stand)
        # leg.front_L(stand)
        # leg.back_R(stand)
        # leg.back_L(stand) #똑바로

        # time.sleep(1)

        leg.back_R(stand)
        time.sleep(0.3)
        leg.back_L(up_out)
        time.sleep(0.3)
        leg.front_R(up_in)
        time.sleep(0.3)

        leg.front_L(up_out)
        time.sleep(0.5)

        leg.front_L(walk)
        time.sleep(2) #왼 앞발
        print("왼 앞발")