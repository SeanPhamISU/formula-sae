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
left_suspension.solve_problem(0, 0)
left_suspension.problem.plot()

# DATA COLLECTION
u_z_offsets = np.linspace(-0.3, 0.3, 5)
u_y_offsets = np.linspace(-0.3, 0.3, 5)
roll_angles = np.linspace(-1.5, 1.5, CHART_ITERATIONS)
camber_results = []

og_U = U

for u_z_offset in u_z_offsets:
    update_u( (og_U[0], og_U[1] + u_z_offset), left_suspension, right_suspension )
    camber_results.append([])
    for roll_angle in roll_angles:
        right_suspension.solve_problem( roll_angle, 0 )
        camber_results[-1].append(right_suspension.get_camber())

# Create 2D meshgrid for 3D plotting
X, Y = np.meshgrid(roll_angles, u_z_offsets)
Z = np.array(camber_results)

# Initialize 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
surf = ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)

# Labels and view angle
ax.set_xlabel('Roll Angle (deg)')
ax.set_ylabel('Vertical Offset u_z (in)')
ax.set_zlabel('Camber Angle (deg)')
ax.set_title('Suspension Camber Gain vs. Roll & Heave')

# Add a color bar map
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Camber (deg)')

plt.show()