from pygeosolve import Problem
import math
from params import *
from suspension import *
import matplotlib.pyplot as plt

right_suspension = Suspension(True)
left_suspension = Suspension(False)
left_suspension.build_geometry()
right_suspension.build_geometry()
right_suspension.solve_problem(0, 0)
left_suspension.solve_problem(0, 0)

camber_resullts = []
for roll_angle in CHASSIS_ROLL_ANGLES:
    right_suspension.solve_problem(roll_angle, 0)
    camber_resullts.append(right_suspension.get_camber())
print(camber_resullts)
plt.plot( CHASSIS_ROLL_ANGLES, camber_resullts )
plt.ylabel("Right Tire Camber")
plt.xlabel("Roll angle")
# debug_errors()
plt.show()
# problem.plot()

print("Data Collection Done")