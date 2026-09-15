from pygeosolve import Problem
import math
from constants import *
from suspension import *
import matplotlib.pyplot as plt

build_geometry()

# # debug_errors()
solve_problem(15, 0)
print(get_camber())
print(problem["Ox"].angle_to(problem["O_TCL"]))
print("Total squared errors: " + str(get_total_errors_squared()))

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
problem.plot()

print("Data Collection Done")