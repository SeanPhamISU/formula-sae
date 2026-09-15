from pygeosolve import Problem
import math
from constants import *
from suspension import *
import matplotlib.pyplot as plt

right_suspension = Suspension(True)
left_suspension = Suspension(False)
left_suspension.build_geometry()
right_suspension.build_geometry()
right_suspension.solve_problem(0, 0)
left_suspension.solve_problem(0, 0)
# right_suspension.debug_errors()
print(str(left_suspension.get_camber()) + "     " + str(right_suspension.get_camber()))
print("Total squared errors: " + str(right_suspension.get_total_errors_squared()))
print( float(right_suspension.get_roll_center(left_suspension).y)  )

left_suspension.problem.plot()

# camber_resullts = []
# for roll_angle in CHASSIS_ROLL_ANGLES:
#     print("Solving for roll angle " + str(roll_angle))
#     solve_problem(roll_angle, 0)
#     camber_resullts.append(get_camber())
# print(camber_resullts)
# plt.plot( CHASSIS_ROLL_ANGLES, camber_resullts )
# plt.ylabel("Right Tire Camber")
# plt.xlabel("Roll angle")
# debug_errors()
# plt.show()
# problem.plot()

print("Data Collection Done")