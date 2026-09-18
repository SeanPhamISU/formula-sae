# INITIAL IMPORTS AND SUSPENSION CREATION
from pygeosolve import Problem
from params import *
from suspension import *
import matplotlib.pyplot as plt
import numpy as np
import params

right_suspension = Suspension(True)
left_suspension = Suspension(False)
left_suspension.build_geometry()
right_suspension.build_geometry()
right_suspension.solve_problem(1.5, 0)
left_suspension.solve_problem(1.5, 0)

# DATA COLLECTION
right_suspension.debug_errors()
print(right_suspension.get_roll_center(left_suspension))
right_suspension.problem.plot()
