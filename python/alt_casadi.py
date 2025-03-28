import casadi as ca
import numpy as np
import json
import csv

# Define joint count
NUM_JOINTS = 3  # Elevator, Elbow, Wrist

# Time horizon
T = 2.0  # seconds
N = 50   # number of discretization steps

# Joint boundaries (example)
joint_pos_min = np.array([0.0, -1.5, -1.0])
joint_pos_max = np.array([2.0, 1.5, 1.0])
joint_vel_max = np.array([1.0, 2.0, 2.0])
joint_acc_max = np.array([2.0, 3.0, 3.0])

# Start and goal
start_pos = np.array([0.0, 0.0, 0.0])
end_pos = np.array([1.5, 1.0, 0.5])

# Time step
dt = T / N

# Optimization variables
opti = ca.Opti()

q = opti.variable(NUM_JOINTS, N+1)  # joint positions
v = opti.variable(NUM_JOINTS, N+1)  # joint velocities
a = opti.variable(NUM_JOINTS, N)    # joint accelerations

# Objective: minimize total acceleration (smoothness)
opti.minimize(ca.sumsqr(a))

# Dynamics constraints
for k in range(N):
    opti.subject_to(q[:, k+1] == q[:, k] + v[:, k]*dt + 0.5*a[:, k]*dt**2)
    opti.subject_to(v[:, k+1] == v[:, k] + a[:, k]*dt)

# Boundary constraints
for j in range(NUM_JOINTS):
    opti.subject_to(opti.bounded(joint_pos_min[j], q[j, :], joint_pos_max[j]))
    opti.subject_to(opti.bounded(-joint_vel_max[j], v[j, :], joint_vel_max[j]))
    opti.subject_to(opti.bounded(-joint_acc_max[j], a[j, :], joint_acc_max[j]))

# Initial and final conditions
opti.subject_to(q[:, 0] == start_pos)
opti.subject_to(q[:, -1] == end_pos)
opti.subject_to(v[:, 0] == 0)
opti.subject_to(v[:, -1] == 0)

# Solver settings
opti.solver('ipopt')
sol = opti.solve()

# Extract trajectory
q_traj = sol.value(q)
v_traj = sol.value(v)

# Export to CSV
with open('trajectory.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['time', 'elevator', 'elbow', 'wrist'])
    for i in range(N+1):
        time = i * dt
        writer.writerow([time] + list(q_traj[:, i]))

# Optionally export to JSON
trajectory = [
    {"time": i * dt, "positions": q_traj[:, i].tolist(), "velocities": v_traj[:, i].tolist()}
    for i in range(N+1)
]
with open("trajectory.json", "w") as f:
    json.dump(trajectory, f, indent=2)
