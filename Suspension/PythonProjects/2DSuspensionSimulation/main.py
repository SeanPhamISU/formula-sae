import pygeosolve as pgs
from pygeosolve import Problem
import math

problem = Problem()

def get_dist(A, B):
    return math.sqrt((A[0] - B[0])**2 + ( A[1] - B[1] )**2)
def rotate_point_at_pivot(P, pv, theta):
    theta = theta * math.pi / 180
    x = ( (P[0] - pv[0]) * math.cos(theta) - (P[1] - pv[1]) * math.sin(theta) ) + pv[0]
    y = ( (P[0] - pv[0]) * math.sin(theta) + (P[1] - pv[1]) * math.cos(theta) ) + pv[1]
    return (x, y)

U = (11.189, 7.315)
UO = (19.75,10.287)
L = (11.354, 2.9235)
LO = (21.432, 3.448)
TC = (23.228, 6.705)
U_UO = get_dist(U, UO)
L_LO = get_dist(L, LO)

UO_TC = get_dist(UO, TC)
LO_TC = get_dist(LO, TC)
UO_LO = get_dist(UO, LO)

TIRE_D = 16
TIRE_RIM_D = 10
TIRE_W = 7.5
TIRE_COMPRESSION = 1.295

CHASSIS_ROLL_POINT = (0, 1.9)
CHASSIS_ROLL_ANGLE = 0

# problem.add_line("upper_arm0", U, UO)
# problem.add_line("lower_arm0", L, LO)

# U = rotate_point_at_pivot(U, CHASSIS_ROLL_POINT, CHASSIS_ROLL_ANGLE)
# L = rotate_point_at_pivot(L, CHASSIS_ROLL_POINT, CHASSIS_ROLL_ANGLE)

#SUSPENSION
problem.add_point("U", U[0], U[1])
problem.add_point("L", L[0], L[1])
problem.add_point("UO", UO[0], UO[1])
problem.add_point("LO", LO[0], LO[1])
problem.add_point("TC", TC[0], TC[1])
problem.add_point("O", 0, 0)

problem.constrain_position("U")
problem.constrain_position("L")
problem.constrain_position("O")

problem.add_line("upper_arm", problem["U"], problem["UO"])
problem.add_line("lower_arm", problem["L"], problem["LO"])
problem.constrain_line_length("upper_arm", U_UO)
problem.constrain_line_length("lower_arm", L_LO)

problem.add_line("tire", (TC[0], 0), (TC[0], TC[1] + TIRE_D/2) )
problem.add_line("tire_UO", problem["tire"].start, problem["UO"])
problem.add_line("tire_LO", problem["tire"].start, problem["LO"])
problem.constrain_angle_between_lines("tire_UO", "tire_LO", 0)
problem.constrain_line_length("tire", TIRE_D - TIRE_COMPRESSION)

# problem.solve()
problem.plot()
print("Done")