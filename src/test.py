from spot_mini_functions import *
motion = motion()

dot = [70, 50, 150]

commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]

while (1):
    motion.oneleg_raise([-20, -10, 0], "front_R", commend)
    motion.oneleg_raise([-20, 10, 0], "front_L", commend)
    motion.oneleg_raise([20, -10, 0], "back_R", commend)
    motion.oneleg_raise([20, 10, 0], "back_L", commend)