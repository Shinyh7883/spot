from spot_mini_functions import *
motion = motion()
control = control()
dot = [0, 20, 120]

commend = [[dot, dot, 2],[dot, dot, 2],[dot, dot, 2],[dot, dot, 2]]
walksp = 400
while (1):
    

    control.commend_set(commend)
    dot1 = motion.dot_move('x', 40, dot)
    print(dot1)
    control.commend(commend, "front_R", dot1, "linear")
    control.commend_run(commend, walksp)

    
    control.commend_set(commend)
    dot1 = motion.dot_move('y', 40, dot1)
    print(dot1)

    control.commend(commend, "front_R", dot1, "linear")
    control.commend_run(commend, walksp)

    
    control.commend_set(commend)
    dot1 = motion.dot_move('x', -40, dot1)
    print(dot1)
    control.commend(commend, "front_R", dot1, "linear")
    control.commend_run(commend, walksp)

    
    control.commend_set(commend)
    dot1 = motion.dot_move('y', -40, dot1)
    print(dot1)
    control.commend(commend, "front_R", dot1, "linear")
    control.commend_run(commend, walksp)