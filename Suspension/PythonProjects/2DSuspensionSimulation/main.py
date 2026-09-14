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

TIRE_D = 16
TIRE_RIM_D = 10
TIRE_W = 7.5

CHASSIS_ROLL_POINT = (0, 1.9)
CHASSIS_ROLL_ANGLE = 3

U = (11.189, 7.315)
UO = (19.75,10.287)
L = (11.354, 2.9235)
LO = (21.432, 3.448)
TC = (23.228, 6.705)
TCU = (TC[0], TC[1] + TIRE_D/2)
TCL = (TC[0], 0)
TIRE_COMPRESSION = math.fabs(TIRE_D/2 - TC[1])

U_UO = get_dist(U, UO)
L_LO = get_dist(L, LO)

UO_TC = get_dist(UO, TC)
LO_TC = get_dist(LO, TC)
UO_LO = get_dist(UO, LO)
UO_TCU = get_dist(UO, TCU)
LO_TCL = get_dist(LO, TCL)
TC_TCU = get_dist(TC, TCU)
TC_TCL = get_dist(TC, TCL)

A = U
B = UO
C = L
D = LO

U = rotate_point_at_pivot(U, CHASSIS_ROLL_POINT, CHASSIS_ROLL_ANGLE)
L = rotate_point_at_pivot(L, CHASSIS_ROLL_POINT, CHASSIS_ROLL_ANGLE)
# print("U position: " + str(U))
# print("Upper arm length: " + str(U_UO))

#SUSPENSION
problem.add_point("U", U[0], U[1])
problem.add_point("L", L[0], L[1])
problem.add_point("UO", UO[0], UO[1])
problem.add_point("LO", LO[0], LO[1])
problem.add_point("TC", TC[0], TC[1])
problem.add_point("TCU", TCU[0], TCU[1])
problem.add_point("TCL", TCL[0], TCL[1])
problem.add_point("O", 0, 0)
problem.add_line("Ox", problem["O"], (20, 0))
problem.add_line("Oy", problem["O"], (0, 20))

problem.constrain_position("U")
problem.constrain_position("L")
problem.constrain_position("O")
problem.constrain_position("Ox")
problem.constrain_position("Oy")

problem.add_line("up_arm", problem["U"], problem["UO"])
problem.add_line("low_arm", problem["L"], problem["LO"])
problem.add_line("UO_LO", problem["UO"], problem["LO"])
problem.add_line("UO_TC", problem["UO"], problem["TC"])
problem.add_line("LO_TC", problem["LO"], problem["TC"])
problem.constrain_line_length("up_arm", U_UO)
problem.constrain_line_length("low_arm", L_LO)
problem.constrain_line_length("UO_LO", UO_LO)
problem.constrain_line_length("UO_TC", UO_TC)
problem.constrain_line_length("LO_TC", LO_TC)
# problem.constrain_angle_between_lines("up_arm", "UO_TC", problem["up_arm"].angle_to(problem["UO_TC"]))
# problem.constrain_angle_between_lines("low_arm", "LO_TC", problem["low_arm"].angle_to(problem["LO_TC"]))

problem.add_line("tire", problem["TCL"], problem["TCU"])
problem.add_line("UO_TCU", problem["UO"], problem["TCU"])
problem.add_line("LO_TCL", problem["LO"], problem["TCL"])
problem.add_line("TC_TCU", problem["TC"], problem["TCU"])
problem.add_line("TC_TCL", problem["TC"], problem["TCL"])
problem.constrain_line_length("UO_TCU", UO_TCU)
problem.constrain_line_length("LO_TCL", LO_TCL)
# problem.constrain_line_angle("UO_LO", "tire", )
problem.constrain_line_length("TC_TCU", TC_TCU)
problem.constrain_line_length("TC_TCL", TC_TCL)
problem.constrain_line_length("tire", TIRE_D-TIRE_COMPRESSION)

problem.add_line("O_TCL", problem["O"], problem["TCL"])
problem.constrain_angle_between_lines("O_TCL", "Ox", -0.0001)

problem.solve()
print("Tire length: " + str(problem["tire"].length()))
print("TCL Position: " + str(problem["TCL"].x) + "  " + str(problem["TCL"].y))
print("Angle: " + str(problem["Ox"].angle_to(problem["O_TCL"])))
print("Camber Angle: " + str(problem["Oy"].angle_to(problem["tire"])))
# print("U position in solution: " + str(problem["U"].x) + " " + str(problem["U"].y))
# print("Upper arm length in solution: " + str(problem["up_arm"].length()))

problem.add_line("upper_arm0", A, B)
problem.add_line("lower_arm0", C, D)
problem.add_line("test_arm0", B, D)
problem.plot()
print("Done")