'''
ROLL ANGLE vs ROLL CENTER HEIGHT vs X OR Y POSITION OF UCA OR LCA INNER POINT
'''

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
right_suspension.solve_problem(0, 0)

# DATA COLLECTION
og_U = U
og_L = L
offsets = np.linspace(-0.3, 0.3, 15)
roll_angles = np.linspace(0.0, 1.5, int(CHART_ITERATIONS/2) )

u_x_positions = []
u_y_positions = []
l_x_positions = []
l_y_positions = []

rc_results_u_x = []
rc_results_u_y = []
rc_results_l_x = []
rc_results_l_y = []

for offset in offsets:
    update_u( (og_U[0] + offset, og_U[1]), left_suspension, right_suspension )
    rc_results_u_x.append([])
    u_x_positions.append(og_U[1] + offset)
    for roll_angle in roll_angles:
        left_suspension.solve_problem(roll_angle, 0)
        right_suspension.solve_problem(roll_angle, 0 )
        rc_results_u_x[-1].append(right_suspension.get_roll_center(left_suspension)[1])
    print("Data collection iteration done")
print("Data collection for Roll angle vs RC Height vs UCA x position complete!")

for offset in offsets:
    update_u( (og_U[0], og_U[1] + offset), left_suspension, right_suspension )
    rc_results_u_y.append([])
    u_y_positions.append(og_U[1] + offset)
    for roll_angle in roll_angles:
        left_suspension.solve_problem(roll_angle, 0)
        right_suspension.solve_problem(roll_angle, 0 )
        rc_results_u_y[-1].append(right_suspension.get_roll_center(left_suspension)[1])
    print("Data collection iteration done")
print("Data collection for Roll angle vs RC Height vs UCA y position complete!")

update_u( (og_U[0], og_U[1]), left_suspension, right_suspension )

for offset in offsets:
    update_l( (og_L[0] + offset, og_L[1]), left_suspension, right_suspension )
    rc_results_l_x.append([])
    l_x_positions.append(og_L[1] + offset)
    for roll_angle in roll_angles:
        left_suspension.solve_problem(roll_angle, 0)
        right_suspension.solve_problem(roll_angle, 0 )
        rc_results_l_x[-1].append(right_suspension.get_roll_center(left_suspension)[1])
    print("Data collection iteration done")
print("Data collection for Roll angle vs RC Height vs LCA x position complete!")

for offset in offsets:
    update_l( (og_L[0], og_L[1] + offset), left_suspension, right_suspension )
    rc_results_l_y.append([])
    l_y_positions.append(og_L[1] + offset)
    for roll_angle in roll_angles:
        left_suspension.solve_problem(roll_angle, 0)
        right_suspension.solve_problem(roll_angle, 0 )
        rc_results_l_y[-1].append(right_suspension.get_roll_center(left_suspension)[1])
    print("Data collection iteration done")
print("Data collection for Roll angle vs RC height vs LCA y position complete!")


# PLOTS
fig, axs = plt.subplots(2, 2, figsize=(12, 9), subplot_kw={'projection': '3d'})

X, Y = np.meshgrid(roll_angles, u_x_positions)
Z = np.array(rc_results_u_x)
surf = axs[0][0].plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)
axs[0][0].set_xlabel('Roll Angle (deg)')
axs[0][0].set_ylabel('UCA X position  (in)')
axs[0][0].set_zlabel('RC Height (in)')
axs[0][0].set_title('UCA X Position vs. Roll vs RC Height')

X, Y = np.meshgrid(roll_angles, u_y_positions)
Z = np.array(rc_results_u_y)
surf = axs[0][1].plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)
axs[0][1].set_xlabel('Roll Angle (deg)')
axs[0][1].set_ylabel('UCA Y position  (in)')
axs[0][1].set_zlabel('RC Height (in)')
axs[0][1].set_title('UCA Y Position vs. Roll vs RC Height')

X, Y = np.meshgrid(roll_angles, l_x_positions)
Z = np.array(rc_results_l_x)
surf = axs[1][0].plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)
axs[1][0].set_xlabel('Roll Angle (deg)')
axs[1][0].set_ylabel('LCA X position  (in)')
axs[1][0].set_zlabel('RC Height (in)')
axs[1][0].set_title('LCA X Position vs. Roll vs RC Height')

X, Y = np.meshgrid(roll_angles, l_y_positions)
Z = np.array(rc_results_l_y)
surf = axs[1][1].plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)
axs[1][1].set_xlabel('Roll Angle (deg)')
axs[1][1].set_ylabel('LCA Y position  (in)')
axs[1][1].set_zlabel('RC Height (in)')
axs[1][1].set_title('LCA Y Position vs. Roll vs RC Height')

plt.show()