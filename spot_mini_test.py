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
    while (1):
        location=[-90, 26, 100]
        theta = kinematics.leg_IK(location)
        leg.front_R(theta)
        leg.front_L(theta)
        leg.back_R(theta)
        leg.back_L(theta)

        time.sleep(1)

        location=[-110, 26, 100]
        theta = kinematics.leg_IK(location)
        leg.front_R(theta)
        leg.front_L(theta)
        leg.back_R(theta)
        leg.back_L(theta)

        time.sleep(1)
