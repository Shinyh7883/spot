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

val_list = [60, 60, 120, 120, 20, 20, 160, 160, 90, 90, 90, 90]

if __name__ == '__main__':
    for x in range(len(val_list)):
        kit[0].servo[x].angle = val_list[x]

    while (1):
        location=[10, 10, 120]
        location[2] = int(input("z위치:"))
        theta = kinematics.leg_IK(location)
        leg.front_R(theta)