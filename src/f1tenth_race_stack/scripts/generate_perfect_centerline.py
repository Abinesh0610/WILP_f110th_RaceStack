#!/usr/bin/env python3
"""
generate_perfect_centerline.py
Generates a mathematically perfect track centerline using straight lines 
and exact circular arcs for the corners, preventing any spline over/undershoot.
"""
import csv
import math
import numpy as np

OUT_CSV = '/home/bits/ABINESH_Packages/racer_ws/src/f1tenth_race_stack/maps/levine_racing_line.csv'

# Corridor centers
TOP_Y = 8.625
BOT_Y = -0.175
LEFT_X = -13.6
RIGHT_X = 9.7

# We want the corner arcs to have a radius of ~0.875m (the half-width of the 1.75m corridors).
# Let's use R = 1.0m to make the curves slightly smoother, which means the turn starts 1.0m before the intersection center.
R = 1.0

# Define the straight segments
# 1. Top straight (going left): from X=(RIGHT_X - R) to X=(LEFT_X + R) at Y=TOP_Y
# 2. Left arc: from (LEFT_X + R, TOP_Y) to (LEFT_X, TOP_Y - R)
# 3. Left straight (going down): from Y=(TOP_Y - R) to Y=(BOT_Y + R) at X=LEFT_X
# 4. Bottom arc: from (LEFT_X, BOT_Y + R) to (LEFT_X + R, BOT_Y)
# 5. Bottom straight (going right): from X=(LEFT_X + R) to X=(RIGHT_X - R) at Y=BOT_Y
# 6. Right arc: from (RIGHT_X - R, BOT_Y) to (RIGHT_X, BOT_Y + R)
# 7. Right straight (going up): from Y=(BOT_Y + R) to Y=(TOP_Y - R) at X=RIGHT_X
# 8. Top-right arc: from (RIGHT_X, TOP_Y - R) to (RIGHT_X - R, TOP_Y)

pts = []
resolution = 0.05

# 1. Top straight (East to West)
x_vals = np.arange(RIGHT_X - R, LEFT_X + R, -resolution)
for x in x_vals:
    pts.append((x, TOP_Y))

# 2. Top-Left Arc (Center: LEFT_X + R, TOP_Y - R)
cx, cy = LEFT_X + R, TOP_Y - R
angles = np.arange(math.pi/2, math.pi, resolution/R)
for a in angles:
    pts.append((cx + R*math.cos(a), cy + R*math.sin(a)))

# 3. Left straight (North to South)
y_vals = np.arange(TOP_Y - R, BOT_Y + R, -resolution)
for y in y_vals:
    pts.append((LEFT_X, y))

# 4. Bottom-Left Arc (Center: LEFT_X + R, BOT_Y + R)
cx, cy = LEFT_X + R, BOT_Y + R
angles = np.arange(math.pi, 3*math.pi/2, resolution/R)
for a in angles:
    pts.append((cx + R*math.cos(a), cy + R*math.sin(a)))

# 5. Bottom straight (West to East)
x_vals = np.arange(LEFT_X + R, RIGHT_X - R, resolution)
for x in x_vals:
    pts.append((x, BOT_Y))

# 6. Bottom-Right Arc (Center: RIGHT_X - R, BOT_Y + R)
cx, cy = RIGHT_X - R, BOT_Y + R
angles = np.arange(3*math.pi/2, 2*math.pi, resolution/R)
for a in angles:
    pts.append((cx + R*math.cos(a), cy + R*math.sin(a)))

# 7. Right straight (South to North)
y_vals = np.arange(BOT_Y + R, TOP_Y - R, resolution)
for y in y_vals:
    pts.append((RIGHT_X, y))

# 8. Top-Right Arc (Center: RIGHT_X - R, TOP_Y - R)
cx, cy = RIGHT_X - R, TOP_Y - R
angles = np.arange(0, math.pi/2, resolution/R)
for a in angles:
    pts.append((cx + R*math.cos(a), cy + R*math.sin(a)))

V_MAX = 3.0
V_MIN = 1.2
CURV_GAIN = 8.0

n = len(pts)
with open(OUT_CSV, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'heading', 'target_speed'])
    for i in range(n):
        j = (i + 1) % n
        prev = (i - 1) % n
        heading = math.atan2(pts[j][1]-pts[i][1], pts[j][0]-pts[i][0])
        
        dx1, dy1 = pts[i][0]-pts[prev][0], pts[i][1]-pts[prev][1]
        dx2, dy2 = pts[j][0]-pts[i][0],   pts[j][1]-pts[i][1]
        cross = abs(dx1*dy2 - dy1*dx2)
        norm  = (math.hypot(dx1,dy1)+1e-9)*(math.hypot(dx2,dy2)+1e-9)
        v = max(V_MIN, min(V_MAX, V_MAX*(1.0 - CURV_GAIN*cross/norm)))
        
        writer.writerow([f'{pts[i][0]:.6f}', f'{pts[i][1]:.6f}',
                         f'{heading:.6f}', f'{v:.4f}'])

stheta = math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
print(f'Saved {n} waypoints')
print(f'First: ({pts[0][0]:.2f}, {pts[0][1]:.2f}), heading={stheta:.3f} rad')
