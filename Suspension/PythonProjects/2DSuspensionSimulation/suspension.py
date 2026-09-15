from pygeosolve import Problem
from constants import *
from utils import * 
from simpy import Point, Line

problem = Problem()

up_arm_constraint = None
low_arm_constraint = None
UO_LO_constraint = None
UO_TC_constraint = None
LO_TC_constraint = None
UO_TCU_constraint = None
LO_TCL_constraint = None
tire_constraint = None

constraints = []

def build_geometry():
    global up_arm_constraint, low_arm_constraint, UO_LO_constraint, UO_TC_constraint, LO_TC_constraint, UO_TCU_constraint, LO_TCL_constraint, tire_constraint

    problem.add_point("U", U[0], U[1])
    problem.add_point("L", L[0], L[1])
    problem.add_point("UO", UO[0], UO[1])
    problem.add_point("LO", LO[0], LO[1])
    problem.add_point("TC", TC[0], TC[1])
    problem.add_point("TCU", TCU[0], TCU[1])
    problem.add_point("TCL", TCL[0], TCL[1])
    problem.add_point("O", 0, 0)
    problem.add_line("Ox", (0, 0), (20, 0))
    problem.add_line("Oy", (0, 0), (0, 20))

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
    # problem.add_line("U_LO", problem["U"], problem["LO"])
    # problem.add_line("L_UO", problem["L"], problem["UO"])
    up_arm_constraint = problem.constrain_line_length("up_arm", U_UO)
    low_arm_constraint = problem.constrain_line_length("low_arm", L_LO)
    UO_LO_constraint = problem.constrain_line_length("UO_LO", UO_LO)
    UO_TC_constraint = problem.constrain_line_length("UO_TC", UO_TC)
    LO_TC_constraint = problem.constrain_line_length("LO_TC", LO_TC)
    # problem.constrain_line_length("U_LO", U_LO)
    # problem.constrain_line_length("L_UO", L_UO)
    problem.constrain_angle_between_lines("UO_TC", "LO_TC", problem["UO_TC"].angle_to(problem["LO_TC"]))
    # problem.constrain_angle_between_lines("up_arm", "UO_TC", problem["up_arm"].angle_to(problem["UO_TC"]))
    # problem.constrain_angle_between_lines("low_arm", "LO_TC", problem["low_arm"].angle_to(problem["LO_TC"]))

    problem.add_line("tire", problem["TCL"], problem["TCU"])
    problem.add_line("UO_TCU", problem["UO"], problem["TCU"])
    problem.add_line("LO_TCL", problem["LO"], problem["TCL"])
    problem.add_line("TC_TCU", problem["TC"], problem["TCU"])
    problem.add_line("TC_TCL", problem["TC"], problem["TCL"])
    UO_TCU_constraint = problem.constrain_line_length("UO_TCU", UO_TCU)
    LO_TCL_constraint = problem.constrain_line_length("LO_TCL", LO_TCL)
    # problem.constrain_angle_between_lines("UO_TCU", "tire", problem["UO_TCU"].angle_to(problem["tire"]))
    problem.constrain_line_length("TC_TCU", TC_TCU)
    problem.constrain_line_length("TC_TCL", TC_TCL)
    tire_constraint = problem.constrain_line_length("tire", TIRE_D-TIRE_COMPRESSION)

    problem.add_line("O_TCL", problem["O"], problem["TCL"])
    problem.constrain_angle_between_lines("O_TCL", "Ox", -0.0001)

    constraints.append(up_arm_constraint)
    constraints.append(low_arm_constraint)
    constraints.append(UO_LO_constraint)
    constraints.append(UO_TC_constraint)
    constraints.append(LO_TC_constraint)
    constraints.append(UO_TCU_constraint)
    constraints.append(LO_TCL_constraint)
    constraints.append(tire_constraint)

def solve_problem(roll_angle, bump):
    problem["U"].position(rotate_point_at_pivot(U, CHASSIS_ROLL_POINT, roll_angle))
    problem["L"].position(rotate_point_at_pivot(L, CHASSIS_ROLL_POINT, roll_angle))
    problem["O"].position( (0, bump) )
    problem["TC"].position( (TC[0], TC[1] + bump) )
    problem["TCL"].position( (TCL[0], TCL[1] + bump) )
    problem["TCU"].position( (TCU[0], TCU[1] + bump) )
    # problem.solve("minimize")
    # problem.solve("bh", niter=2)
    problem.solve(
        "minimize", 
        method='SLSQP', 
        options={
            'ftol': 1e-15, 
            'maxiter': 25
        }
    )
    # problem.solve("ls", ftol=1e-12, xtol=1e-12, gtol=1e-12)
    return problem

def get_tire_to_ground_height():
    angle = problem["Ox"].angle_to(problem["O_TCL"])
    l = problem["O_TCL"].length()
    return l * math.sin(angle * math.pi / 180)
def get_camber():
    return problem["Oy"].angle_to(problem["tire"])

def get_total_errors_squared():
    error = 0
    for constraint in constraints:
        error += constraint.error()**2
    return error

def get_roll_center():
    UUO = Line(Point(problem["U"].x, problem["U"].y), Point(problem["UO"].x, problem["UO"].y))
    LLO = Line(Point(problem["U"].x, problem["U"].y), Point(problem["UO"].x, problem["UO"].y))
    # WIP because roll_center require left suspension too

def debug_errors():
    print("================ CONSTRAINT ERRORS ================")
    print("Upper arm constraint error: " + str(up_arm_constraint.error()))
    print("Lower arm constraint error: " + str(low_arm_constraint.error()))
    print("UO_LO constraint error: " + str(UO_LO_constraint.error()))
    print("UO_TC constraint error: " + str(UO_TC_constraint.error()))
    print("LO_TC constraint error: " + str(LO_TC_constraint.error()))
    print("UO_TCU constraint error: " + str(UO_TCU_constraint.error()))
    print("LO_TCL constraint error: " + str(LO_TCL_constraint.error()))
    print("Tire constraint error: " + str(tire_constraint.error()))
