from spot_mini_functions import *
import numpy as np
control = control()
functions = functions()
dot1 = [0, 20, 140]
dot2 = [0, 20, 80]
dot3 = [0, 20, 140]


commend = [[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2]]


while 1:

    control.commend_set(commend)
    control.commend(commend, "front_R", dot2, "direct")
    control.commend(commend, "back_R", dot2, "direct")
    control.commend(commend, "front_L", dot2, "direct")
    control.commend(commend, "back_L", dot2, "direct")
    control.commend_run(commend, 500)

    control.commend_set(commend)
    control.commend(commend, "front_R", dot1, "direct")
    control.commend(commend, "back_R", dot1, "direct")
    control.commend(commend, "front_L", dot1, "direct")
    control.commend(commend, "back_L", dot1, "direct")
    control.commend_run(commend, 500)
