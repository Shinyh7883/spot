from spot_mini_functions import *
functions = functions()

dot1 = [60, 50, 170]
dot2 = [70, 50, 170]

commend = [[dot1, dot2, 2],[dot1, dot2, 2],[dot1, dot2, 2],[dot1, dot2, 2]]

Dt = functions.fifthpoly(1, commend[0][0], commend[0][1])
print(Dt)
