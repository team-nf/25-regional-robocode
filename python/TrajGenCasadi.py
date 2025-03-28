import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation
import json

from casadi import *


constraints = {}

# set up solver
opti = Opti()
opti.solver("ipopt")

# config
n = 4 # interior points

origin = (0, 0)

elevator_min_length = 0
elevator_length = 0
elevator_max_length = 14
shoulder_length = 3.75
elbow_length = 3.75

max_voltage_elevator = 10
max_voltage_shoulder = 10
max_voltage_elbow = 9

max_accel_elevator = 81
max_accel_shoulder = 180
max_accel_elbow = 130

shoulder_cg_radius = 0
elbow_cg_radius = 0

shoulder_min_angle = 0
shoulder_max_angle = 0
elbow_min_angle = 0
elbow_max_angle = 0

shoulder_moi = 0

# time variables
total_time = opti.variable()
dt = total_time / (n + 1) # OF

# theta points
theta_points = []
theta_points.append([opti.parameter(), opti.parameter()])

for _ in range(n):
    theta_0  = opti.variable()
    theta_1 = opti.variable()
    opti.subject_to(
        opti.bounded(
            shoulder_min_angle,
            theta_0,
            shoulder_max_angle
        )
    )
    opti.subject_to(
        opti.bounded(
            elbow_min_angle,
            theta_1,
            elbow_max_angle
        )
    )
    theta_points.append([theta_0, theta_1])
theta_points.append([opti.parameter(), opti.parameter()])

# create constraint parameters
constraint_parameters = {}
for (constraint_key, constraint) in constraints.items():
    constraints[constraint_key] = opti.parameter()

    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

def solve(parameters):
    # update position parameters
    opti.set_value(
        theta_points[0][0],
        clamp(
            parameters["initial"][0],
            shoulder_min_angle,
            shoulder_max_angle
        ),
    )
    opti.set_value(
        theta_points[0][1],
        clamp(
             parameters["initial"][1],
             elbow_min_angle,
             elbow_max_angle
        )
    )
    opti.set_value(
        theta_points[len(theta_points) -1][0],
        clamp(
            parameters["final"][0],
            shoulder_min_angle,
            shoulder_max_angle
        )
    )
    opti.set_value(
        theta_points[len(theta_points) -1][1],
        clamp(
            parameters["final"][1],
            elbow_min_angle,
            elbow_max_angle)
    )

    opti.set_initial(total_time, 1)
    n = len(theta_points) - 2
    for i in range (1, n+1):
        opti.set_initial(
            theta_points[i][0],
            (parameters["final"][0] - parameters["initial"][0]) * (i / (n + 2))
            + parameters["initial"][0],
        )
        opti.set_initial(
            theta_points[i][1],
            (parameters["final"][1] - parameters["initial"][1]) * (i / (n + 2))
            + parameters["initial"][1],
        )

    # set constraint parameters
    initial_theta = (opti.value(theta_points[0][0]), opti.value(theta_points[0][1]))
    final_theta = (opti.value(theta_points[-1][0]), opti.value(theta_points[-1][1]))
    initial_elev = 0
    final_elev = 9
    start_x = (
        origin[0] + shoulder_length * cos(initial_theta[0]) + elbow_length * cos(initial_theta[0] + initial_theta[1])
    )
    start_y = (
        origin[1] + initial_elev + shoulder_length * sin(initial_theta[0]) + elbow_length * sin(initial_theta[0] + initial_theta[1])
    )
    final_x = (
        origin[0] + shoulder_length * cos(final_theta[0]) + elbow_length * cos(final_theta[0] + final_theta[1])
    )
    final_y = (
        origin[1] + final_elev + shoulder_length * sin(final_theta[0]) + elbow_length * sin(final_theta[0] + final_theta[1])
    )