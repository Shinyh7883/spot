from spot_mini_functions import *
motion = motion()

run = 1

dot = [55, 50, 170]
commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]
# motion.body_rotate(20, commend)
while (1):
    # motion.body_rotate(-40, commend)
    # motion.body_rotate(40, commend)
    motion.right(commend, run)
    motion.left(commend, run)

motion.foward(commend, run)
motion.backward(commend, run)