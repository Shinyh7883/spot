from spot_mini_functions import *
motion = motion()

run = 1
B = -30
state = 1

dot = [55, 50, 170]
commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]
motion.right(commend, run)
state = motion.ready(B, state, commend, run)

while (1):
    # motion.right(commend, run)
    # motion.left(commend, run)
    # motion.foward(commend, run)
    # motion.backward(commend, run)
    # motion.foward_test(commend, run) #망함,,, 앞이 캄캄쓰

    
    state = motion.fbmove(B, state, commend, run)
    state = motion.stop(B, state, commend, run)
    motion.right(commend, run)
    state = motion.ready(B, state, commend, run)

    # B = -B
    # state = motion.fbchange_ready(B, state, commend, run)
    # state = motion.fbmove(B, state, commend, run)
    # state = motion.stop(B, state, commend, run)
    # B = -B
    
    

