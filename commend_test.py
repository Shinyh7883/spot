from spot_mini_functions import *
import numpy as np
control = control()
dot1 = [-20, 20, 100]
dot2 = [-50, 0, 110]
dot3 = [-50, 25, 110]

commend = [[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2],[dot1, dot1, 2]]

control.commend_set(commend)
control.commend(commend, "front_R", dot2, "direct")
control.commend(commend, "back_R", dot3, "direct")
control.commend_run(commend)


control.commend_set(commend)
control.commend(commend, "front_R", dot3, "direct")
control.commend(commend, "back_R", dot1, "direct")
control.commend_run(commend)
