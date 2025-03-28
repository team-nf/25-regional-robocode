import math
import json
import numpy as np
from casadi import *

# Segment count
poses = [
    [-math.pi / 4, math.pi / 6, 0.0],     # Pose A
    [0.0, 0.0, 8.0],                      # Pose B
    [math.pi / 6, -math.pi / 4, 10.0],    # Pose C
]

# Settings
n_per_segment = 30
dt = 0.05
total_steps = n_per_segment * (len(poses) - 1)

# Robot dimensions
shoulder_length = 3.75
elbow_length = 3.75
elevator_min = 0
elevator_max = 14
shoulder_min = -math.pi / 2
shoulder_max = math.pi / 2
elbow_min = -math.pi / 2
elbow_max = math.pi / 2
max_vel = 3.0
max_acc = 6.0

# CasADi
opti = Opti()
theta = opti.variable(total_steps + 1)
phi = opti.variable(total_steps + 1)
h = opti.variable(total_steps + 1)
dtheta = opti.variable(total_steps + 1)
dphi = opti.variable(total_steps + 1)
dh = opti.variable(total_steps + 1)
ddtheta = opti.variable(total_steps)
ddphi = opti.variable(total_steps)
ddh = opti.variable(total_steps)

# Smoothness cost
cost = sumsqr(ddtheta) + sumsqr(ddphi) + sumsqr(ddh)

# Initial conditions
opti.subject_to(theta[0] == poses[0][0])
opti.subject_to(phi[0] == poses[0][1])
opti.subject_to(h[0] == poses[0][2])
opti.subject_to(dtheta[0] == 0)
opti.subject_to(dphi[0] == 0)
opti.subject_to(dh[0] == 0)

# Final conditions
opti.subject_to(theta[-1] == poses[-1][0])
opti.subject_to(phi[-1] == poses[-1][1])
opti.subject_to(h[-1] == poses[-1][2])
opti.subject_to(dtheta[-1] == 0)
opti.subject_to(dphi[-1] == 0)
opti.subject_to(dh[-1] == 0)

# Apply dynamics + constraints
for i in range(total_steps):
    # Euler integration
    opti.subject_to(theta[i+1] == theta[i] + dtheta[i]*dt + 0.5*ddtheta[i]*dt**2)
    opti.subject_to(phi[i+1] == phi[i] + dphi[i]*dt + 0.5*ddphi[i]*dt**2)
    opti.subject_to(h[i+1] == h[i] + dh[i]*dt + 0.5*ddh[i]*dt**2)
    opti.subject_to(dtheta[i+1] == dtheta[i] + ddtheta[i]*dt)
    opti.subject_to(dphi[i+1] == dphi[i] + ddphi[i]*dt)
    opti.subject_to(dh[i+1] == dh[i] + ddh[i]*dt)

    # Bounds
    opti.subject_to(opti.bounded(shoulder_min, theta[i], shoulder_max))
    opti.subject_to(opti.bounded(elbow_min, phi[i], elbow_max))
    opti.subject_to(opti.bounded(elevator_min, h[i], elevator_max))
    opti.subject_to(opti.bounded(-max_vel, dtheta[i], max_vel))
    opti.subject_to(opti.bounded(-max_vel, dphi[i], max_vel))
    opti.subject_to(opti.bounded(-max_vel, dh[i], max_vel))
    opti.subject_to(opti.bounded(-max_acc, ddtheta[i], max_acc))
    opti.subject_to(opti.bounded(-max_acc, ddphi[i], max_acc))
    opti.subject_to(opti.bounded(-max_acc, ddh[i], max_acc))

# Enforce position + velocity continuity at waypoints
for seg in range(1, len(poses) - 1):
    idx = seg * n_per_segment
    tolerance = 0.1  # radians or meters
    opti.subject_to(opti.bounded(poses[seg][0] - tolerance, theta[idx], poses[seg][0] + tolerance))
    opti.subject_to(opti.bounded(poses[seg][0] - tolerance, theta[idx], poses[seg][0] + tolerance)
    opti.subject_to(opti.bounded(poses[seg][0] - tolerance, theta[idx], poses[seg][0] + tolerance))

# Minimize cost
opti.minimize(cost)
opti.solver("ipopt")
sol = opti.solve()

# Extract values
theta_vals = sol.value(theta)
phi_vals = sol.value(phi)
h_vals = sol.value(h)
steps = total_steps + 1

# Forward kinematics
x_vals = shoulder_length * np.cos(theta_vals) + elbow_length * np.cos(theta_vals + phi_vals)
y_vals = h_vals + shoulder_length * np.sin(theta_vals) + elbow_length * np.sin(theta_vals + phi_vals)

# Export format with timestamps
traj_data = {
    "time": {str(i): float(i * dt) for i in range(steps)},
    "theta": {str(i): float(theta_vals[i]) for i in range(steps)},
    "phi": {str(i): float(phi_vals[i]) for i in range(steps)},
    "h": {str(i): float(h_vals[i]) for i in range(steps)},
    "x": {str(i): float(x_vals[i]) for i in range(steps)},
    "y": {str(i): float(y_vals[i]) for i in range(steps)}
}

with open("trajectory.json", "w") as f:
    json.dump(traj_data, f, indent=2)

print("✅ Multi-pose trajectory exported to 'trajectory.json'")
