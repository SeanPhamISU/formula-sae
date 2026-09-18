import math
from utils import *
import numpy as np

CHART_ITERATIONS = 15

TIRE_D = 16
TIRE_RIM_D = 10
TIRE_W = 7.5

CHASSIS_ROLL_POINT = (0, 1.9)

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

def update_u(new_U, left_suspension, right_suspension):
    global U, U_UO
    U = new_U
    U_UO = get_dist(U, UO)
    left_suspension.up_arm_constraint.set_length(U_UO)
    right_suspension.up_arm_constraint.set_length(U_UO)

def update_l(new_L, left_suspension, right_suspension):
    global L, L_LO
    L = new_L
    L_LO = get_dist(L, LO)
    left_suspension.low_arm_constraint.set_length(L_LO)
    right_suspension.low_arm_constraint.set_length(L_LO)