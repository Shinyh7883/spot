import numpy as np
import math
from adafruit_servokit import ServoKit
import board
import busio
import time
from spot_mini_functions import *

functions = functions()

class controll:
    def linear_control(leg, dot1, dot2):
        theta = np.zeros[1, 3]
        dots = functions.trajectory(dot1, dot2)
        
        for i in range(np.shape(dots)[0]): #0 아님 1 이다
            theta = np.append(functions.leg_IK(dots[i]))

        
