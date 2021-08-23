from spot_mini_functions import *
motion = motion()

run = 1

dot = [55, 50, 170]
commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]

while (1):
    motion.right(commend, run)
    motion.left(commend, run)
    motion.foward(commend, run)
    motion.backward(commend, run)
    # motion.foward_test(commend, run) #망함,,, 앞이 캄캄쓰
    

