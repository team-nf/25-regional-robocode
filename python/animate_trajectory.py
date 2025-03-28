import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import numpy as np

shoulder_length = 3.75
elbow_length = 3.75

# Load trajectory
with open("trajectory.json", "r") as f:
    data = json.load(f)

steps = len(data["theta"])
theta = [data["theta"][str(i)] for i in range(steps)]
phi = [data["phi"][str(i)] for i in range(steps)]
h = [data["h"][str(i)] for i in range(steps)]

# Forward kinematics
def fk(i):
    t = theta[i]
    p = phi[i]
    elev = h[i]

    x0, y0 = 0, elev
    x1 = x0 + shoulder_length * np.cos(t)
    y1 = y0 + shoulder_length * np.sin(t)
    x2 = x1 + elbow_length * np.cos(t + p)
    y2 = y1 + elbow_length * np.sin(t + p)
    return [(x0, y0), (x1, y1), (x2, y2)]

# Plot setup
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(0, 20)
ax.set_aspect("equal")
ax.set_title("Blended Multi-Pose Arm Trajectory")
line, = ax.plot([], [], 'o-', lw=4)
gripper_dot, = ax.plot([], [], 'ro', markersize=8)

def update(frame):
    (x0, y0), (x1, y1), (x2, y2) = fk(frame)
    line.set_data([x0, x1, x2], [y0, y1, y2])
    gripper_dot.set_data([x2], [y2])
    return line, gripper_dot

ani = animation.FuncAnimation(fig, update, frames=steps, interval=50, blit=True)
plt.show()
