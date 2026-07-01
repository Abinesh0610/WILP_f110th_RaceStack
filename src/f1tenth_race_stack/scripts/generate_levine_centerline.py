#!/usr/bin/env python3
"""
Final correct Levine centerline using exact measured coordinates.
Track corridor centers measured from levine.png:
  - Top corridor:    Y = 8.6
  - Bottom corridor: Y = -0.15
  - Left corridor:   X = -13.6
  - Right corridor:  X = 9.7

The track is an oval driving counter-clockwise.
"""
import csv, math, os
import numpy as np
from scipy.interpolate import splprep, splev

MAPS_DIR = os.path.expanduser('~/ABINESH_Packages/racer_ws/src/f1tenth_race_stack/maps')
OUT_CSV = os.path.join(MAPS_DIR, 'levine_racing_line.csv')

V_MAX = 3.0
V_MIN = 1.0
CURV_GAIN = 8.0
N_PTS = 500

TOP_Y   =  8.6
BOT_Y   = -0.15
LEFT_X  = -13.6
RIGHT_X =  9.7

# Counter-clockwise: start at top-right, go left across top,
# around left end, right across bottom, around right end, back
keypoints = [
    # Top corridor (going LEFT)
    ( 6.0, TOP_Y),
    ( 1.0, TOP_Y),
    (-4.0, TOP_Y),
    (-9.0, TOP_Y),
    
    # EXACT CORNER to pull the spline outward
    (LEFT_X, TOP_Y),

    # Left turn (around left end)
    (LEFT_X,  4.25),
    
    # EXACT CORNER
    (LEFT_X, BOT_Y),

    # Bottom corridor (going RIGHT)
    (-9.0, BOT_Y),
    (-4.0, BOT_Y),
    ( 1.0, BOT_Y),
    ( 6.0, BOT_Y),
    
    # EXACT CORNER
    (RIGHT_X, BOT_Y),

    # Right turn (around right end)
    (RIGHT_X,  4.25),
    
    # EXACT CORNER
    (RIGHT_X, TOP_Y),

    # Back to top
    ( 6.0, TOP_Y),
]

xs = [p[0] for p in keypoints]
ys = [p[1] for p in keypoints]

# Fit spline
tck, u = splprep([xs, ys], s=0, per=True, k=3)
u_new = np.linspace(0, 1, N_PTS, endpoint=False)
cx, cy = splev(u_new, tck)

with open(OUT_CSV, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'heading', 'target_speed'])
    n = len(cx)
    for i in range(n):
        j = (i + 1) % n
        prev = (i - 1) % n
        heading = math.atan2(cy[j]-cy[i], cx[j]-cx[i])
        
        dx1, dy1 = cx[i]-cx[prev], cy[i]-cy[prev]
        dx2, dy2 = cx[j]-cx[i],   cy[j]-cy[i]
        cross = abs(dx1*dy2 - dy1*dx2)
        norm  = (math.hypot(dx1,dy1)+1e-9)*(math.hypot(dx2,dy2)+1e-9)
        v = max(V_MIN, min(V_MAX, V_MAX*(1.0 - CURV_GAIN*cross/norm)))
        
        writer.writerow([f'{cx[i]:.6f}', f'{cy[i]:.6f}',
                         f'{heading:.6f}', f'{v:.4f}'])

stheta = math.atan2(cy[1]-cy[0], cx[1]-cx[0])
print(f'Saved {N_PTS} waypoints')
print(f'First: ({cx[0]:.2f}, {cy[0]:.2f}), heading={stheta:.3f} rad')
print(f'\nsim.yaml spawn:')
print(f'  sx: {cx[0]:.3f}')
print(f'  sy: {cy[0]:.3f}')
print(f'  stheta: {stheta:.3f}')
