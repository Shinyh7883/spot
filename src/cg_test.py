from spot_mini_functions import *
motion = motion()
functions = functions()
control = control()

run = 1

dot = [65, 50, 170]
commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]

while (1):
    motion.body_move([-20, 0], commend)
    walksp = 400
    front_R = functions.dot_move('z', -20, commend[0][0])
    control.commend(commend, "front_R", front_R, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend)

    print(commend)

    front_R = functions.dot_move('z', 20, commend[0][0])
    control.commend(commend, "front_R", front_R, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend)

    front_L = functions.dot_move('z', -20, commend[1][0])
    control.commend(commend, "front_L", front_L, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend)

    front_L = functions.dot_move('z', 20, commend[1][0])
    control.commend(commend, "front_L", front_L, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend) 
    
    motion.body_move([40, 0], commend)

    back_R = functions.dot_move('z', -20, commend[2][0])
    control.commend(commend, "back_R", back_R, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend)

    back_R = functions.dot_move('z', 20, commend[2][0])
    control.commend(commend, "back_R", back_R, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend) 

    back_L = functions.dot_move('z', -20, commend[3][0])
    control.commend(commend, "back_L", back_L, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend) 

    back_L = functions.dot_move('z', 20, commend[3][0])
    control.commend(commend, "back_L", back_L, "linear")
    control.commend_run(commend, 100)
    control.commend_set(commend) 

    motion.body_move([-20, 0], commend)
