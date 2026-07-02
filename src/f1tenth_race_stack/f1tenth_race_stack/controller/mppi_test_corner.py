import numpy as np
from mppi_controller import MPPIController

import rclpy
rclpy.init()
c = MPPIController()

# Set mock state at a corner. 
# Car is at (0, 0), facing +Y (yaw = pi/2).
c._state = np.array([0.0, 0.0, np.pi/2, 3.0])

# Racing line comes from Y=-5 to Y=0, then turns left to X=-5
pts = []
for y in np.linspace(-5, 0, 10):
    pts.append([0.0, y])
for x in np.linspace(0, -5, 10)[1:]:
    pts.append([x, 0.0])
pts = np.array(pts)

diffs = np.diff(pts, axis=0)
headings = np.arctan2(diffs[:, 1], diffs[:, 0])
headings = np.append(headings, headings[-1])

c._racing_line = np.column_stack([pts, headings])
c._obstacle_points = np.array([[100.0, 100.0]]) # No obstacles

# Test rollout 1: Go straight (delta=0)
u_straight = np.zeros((1, c.T, 2))
u_straight[0, :, 0] = 3.0
u_straight[0, :, 1] = 0.0

# Test rollout 2: Turn left (delta=0.43)
u_left = np.zeros((1, c.T, 2))
u_left[0, :, 0] = 3.0
u_left[0, :, 1] = 0.43

cost_straight = c._rollout(c._state[:3], u_straight)
cost_left = c._rollout(c._state[:3], u_left)

print(f"Cost Straight: {cost_straight[0]}")
print(f"Cost Left:     {cost_left[0]}")

rclpy.shutdown()
