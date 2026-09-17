from pygeosolve import Problem
import params
from params import *
from utils import * 
import time

from sympy.geometry import *

class Suspension:
    def __init__(self, isRight):    
        self.problem = Problem()

        self.up_arm_constraint = None
        self.low_arm_constraint = None
        self.UO_LO_constraint = None
        self.UO_TC_constraint = None
        self.LO_TC_constraint = None
        self.UO_TCU_constraint = None
        self.LO_TCL_constraint = None
        self.tire_constraint = None
        self.constraints = []
        self.mul = 1
        if(not isRight): 
            self.mul = -1

    def build_geometry(self):
        start = time.time()
        self.problem.add_point("U", params.U[0] * self.mul, params.U[1])
        self.problem.add_point("L", params.L[0] * self.mul, params.L[1])
        self.problem.add_point("UO", params.UO[0] * self.mul, params.UO[1])
        self.problem.add_point("LO", params.LO[0] * self.mul, params.LO[1])
        self.problem.add_point("TC", params.TC[0] * self.mul, params.TC[1])
        self.problem.add_point("TCU", params.TCU[0] * self.mul, params.TCU[1])
        self.problem.add_point("TCL", params.TCL[0] * self.mul, params.TCL[1])
        self.problem.add_point("O", 0, 0)
        self.problem.add_line("Ox", (0, 0), (20 * self.mul, 0))
        self.problem.add_line("Oy", (0, 0), (0, 20))

        self.problem.constrain_position("U")
        self.problem.constrain_position("L")
        self.problem.constrain_position("O")
        self.problem.constrain_position("Ox")
        self.problem.constrain_position("Oy")

        self.problem.add_line("up_arm", self.problem["U"], self.problem["UO"])
        self.problem.add_line("low_arm", self.problem["L"], self.problem["LO"])
        self.problem.add_line("UO_LO", self.problem["UO"], self.problem["LO"])
        self.problem.add_line("UO_TC", self.problem["UO"], self.problem["TC"])
        self.problem.add_line("LO_TC", self.problem["LO"], self.problem["TC"])
        self.up_arm_constraint = self.problem.constrain_line_length("up_arm", params.U_UO)
        self.low_arm_constraint = self.problem.constrain_line_length("low_arm", params.L_LO)
        self.UO_LO_constraint = self.problem.constrain_line_length("UO_LO", params.UO_LO)
        self.UO_TC_constraint = self.problem.constrain_line_length("UO_TC", params.UO_TC)
        self.LO_TC_constraint = self.problem.constrain_line_length("LO_TC", params.LO_TC)
        # self.problem.constrain_angle_between_lines("UO_TC", "LO_TC", self.problem["UO_TC"].angle_to(self.problem["LO_TC"]))

        self.problem.add_line("tire", self.problem["TCL"], self.problem["TCU"])
        self.problem.add_line("UO_TCU", self.problem["UO"], self.problem["TCU"])
        self.problem.add_line("LO_TCL", self.problem["LO"], self.problem["TCL"])
        self.problem.add_line("TC_TCU", self.problem["TC"], self.problem["TCU"])
        self.problem.add_line("TC_TCL", self.problem["TC"], self.problem["TCL"])
        self.UO_TCU_constraint = self.problem.constrain_line_length("UO_TCU", params.UO_TCU)
        self.LO_TCL_constraint = self.problem.constrain_line_length("LO_TCL", params.LO_TCL)
        # problem.constrain_angle_between_lines("UO_TCU", "tire", problem["UO_TCU"].angle_to(problem["tire"]))
        self.problem.constrain_line_length("TC_TCU", params.TC_TCU)
        self.problem.constrain_line_length("TC_TCL", params.TC_TCL)
        self.tire_constraint = self.problem.constrain_line_length("tire", params.TIRE_D-params.TIRE_COMPRESSION)

        self.problem.add_line("O_TCL", self.problem["O"], self.problem["TCL"])
        self.problem.constrain_angle_between_lines("O_TCL", "Ox", -0.0001)

        self.constraints.append(self.up_arm_constraint)
        self.constraints.append(self.low_arm_constraint)
        self.constraints.append(self.UO_LO_constraint)
        self.constraints.append(self.UO_TC_constraint)
        self.constraints.append(self.LO_TC_constraint)
        self.constraints.append(self.UO_TCU_constraint)
        self.constraints.append(self.LO_TCL_constraint)
        self.constraints.append(self.tire_constraint)

        end = time.time()
        print("Time to initialize suspension geometry: " + str(end-start))

    def solve_problem(self, roll_angle, bump):
        start = time.time()
        self.problem["U"].position(rotate_point_at_pivot((params.U[0] * self.mul, params.U[1]), CHASSIS_ROLL_POINT, roll_angle))
        self.problem["L"].position(rotate_point_at_pivot((params.L[0] * self.mul, params.L[1]), CHASSIS_ROLL_POINT, roll_angle))
        self.problem["O"].position( (0, bump) )
        self.problem["TC"].position( (TC[0] * self.mul, TC[1] + bump) )
        self.problem["TCL"].position( (TCL[0] * self.mul, TCL[1] + bump) )
        self.problem["TCU"].position( (TCU[0] * self.mul, TCU[1] + bump) )
        # problem.solve("minimize")
        # problem.solve("bh", niter=2)
        self.problem.solve(
            "minimize", 
            method='SLSQP', 
            options={
                'ftol': 1e-12, 
                'maxiter': 25
            }
        )
        end = time.time()
        # print("Time to solve: " + str(end-start))
        # problem.solve("ls", ftol=1e-12, xtol=1e-12, gtol=1e-12)
        return self.problem

    def get_tire_to_ground_height(self):
        angle = self.problem["Ox"].angle_to(self.problem["O_TCL"])
        l = self.problem["O_TCL"].length()
        return l * math.sin(angle * math.pi / 180)
    def get_camber(self):
        return self.problem["Oy"].angle_to(self.problem["tire"]) * self.mul

    def get_total_errors_squared(self):
        error = 0
        for constraint in self.constraints:
            error += constraint.error()**2
        return error

    def get_roll_center(self, other):
        UUO = Line(Point(self.problem["U"].x, self.problem["U"].y), Point(self.problem["UO"].x, self.problem["UO"].y))
        LLO = Line(Point(self.problem["L"].x, self.problem["L"].y), Point(self.problem["LO"].x, self.problem["LO"].y))
        TCLP = Point(self.problem["TCL"].x, self.problem["TCL"].y)

        UUO2 = Line(Point(other.problem["U"].x, other.problem["U"].y), Point(other.problem["UO"].x, other.problem["UO"].y))
        LLO2 = Line(Point(other.problem["L"].x, other.problem["L"].y), Point(other.problem["LO"].x, other.problem["LO"].y))
        TCLP2 = Point(other.problem["TCL"].x, other.problem["TCL"].y)

        S1 = intersection(UUO, LLO)[0]
        S2 = intersection(UUO2, LLO2)[0]
        S1TCL = Line(S1, TCLP)
        S2TCL2 = Line(S2, TCLP2)
        RC = intersection(S1TCL, S2TCL2)[0]
        return RC

    def debug_errors(self):
        print("================ CONSTRAINT ERRORS ================")
        print("Upper arm constraint error: " + str(self.up_arm_constraint.error()))
        print("Lower arm constraint error: " + str(self.low_arm_constraint.error()))
        print("UO_LO constraint error: " + str(self.UO_LO_constraint.error()))
        print("UO_TC constraint error: " + str(self.UO_TC_constraint.error()))
        print("LO_TC constraint error: " + str(self.LO_TC_constraint.error()))
        print("UO_TCU constraint error: " + str(self.UO_TCU_constraint.error()))
        print("LO_TCL constraint error: " + str(self.LO_TCL_constraint.error()))
        print("Tire constraint error: " + str(self.tire_constraint.error()))
