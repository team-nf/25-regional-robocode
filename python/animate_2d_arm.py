import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import json

# Robot parameters
ELEVATOR_BASE = 0.25  # m
L1 = 0.40             # elbow to wrist
L2 = 0.30             # wrist to gripper tip

# Load trajectory
with open("trajectory.json", "r") as f:
    traj = json.load(f)

# Precompute all joint angles and end effector positions
positions = np.array([pt["positions"] for pt in traj])
elevator = positions[:, 0]
theta1 = positions[:, 1]  # Elbow angle (radians)
theta2 = positions[:, 2]  # Wrist angle (radians)

# Forward kinematics function
def compute_fk(elev, t1, t2):
    base_y = ELEVATOR_BASE + elev
    x0, y0 = 0, base_y
    x1 = x0 + L1 * np.cos(t1)
    y1 = y0 + L1 * np.sin(t1)
    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)
    return (x0, y0), (x1, y1), (x2, y2)

# Create figure
fig, ax = plt.subplots()
ax.set_xlim(-1, 1)
ax.set_ylim(0, 2.5)
ax.set_aspect("equal")
line, = ax.plot([], [], "o-", lw=4)
gripper_dot, = ax.plot([], [], "ro", markersize=8)

# Animation update
def update(frame):
    (x0, y0), (x1, y1), (x2, y2) = compute_fk(elevator[frame], theta1[frame], theta2[frame])
    line.set_data([x0, x1, x2], [y0, y1, y2])
    gripper_dot.set_data([x2], [y2])
    return line, gripper_dot

ani = animation.FuncAnimation(fig, update, frames=len(traj), interval=50, blit=True)
plt.title("2D Arm Forward Kinematics")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid()
plt.show()
