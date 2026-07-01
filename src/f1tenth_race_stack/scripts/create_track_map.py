#!/usr/bin/env python3
"""
create_track_map.py
====================
Creates a proper closed-loop binary map from levine.png for racing line generation.

The levine.png map has thin black wall lines on a white background. 
This script:
1. Thickens the black walls using dilation
2. Flood-fills from outside to black out the exterior
3. Saves a cleaned track map where ONLY the track corridor is white

Run from workspace:
  python3 create_track_map.py
"""

import cv2
import numpy as np
import os

MAPS_DIR = os.path.expanduser('~/ABINESH_Packages/racer_ws/src/f1tenth_race_stack/maps')
SRC_MAP = os.path.expanduser('~/ABINESH_Packages/racer_ws/src/f1tenth_gym_ros/maps/levine.png')
OUT_PNG = os.path.join(MAPS_DIR, 'levine_track.png')
OUT_YAML = os.path.join(MAPS_DIR, 'levine_track.yaml')

img = cv2.imread(SRC_MAP, cv2.IMREAD_GRAYSCALE)
print(f'Loaded: {img.shape}, dtype={img.dtype}')

# Step 1: Create binary wall mask (pixels < 50 = wall)
walls = (img < 50).astype(np.uint8) * 255

# Step 2: Thicken walls with dilation so flood-fill cannot leak through thin gaps
kernel = np.ones((5, 5), np.uint8)
thick_walls = cv2.dilate(walls, kernel, iterations=2)

# Step 3: Create a fresh canvas and flood-fill the exterior to black
# The idea: exterior starts black (walls), interior (track corridor) stays white
canvas = np.where(thick_walls > 127, np.uint8(0), np.uint8(255))

# Flood-fill exterior from the 4 corners to mark it as black
h, w = canvas.shape
# Use a mask that is 2 pixels larger on each side (flood-fill requirement)
mask = np.zeros((h + 2, w + 2), np.uint8)
# Fill from top-left corner (exterior)
cv2.floodFill(canvas, mask, (0, 0), 0)
cv2.floodFill(canvas, mask, (w - 1, 0), 0)
cv2.floodFill(canvas, mask, (0, h - 1), 0)
cv2.floodFill(canvas, mask, (w - 1, h - 1), 0)

print(f'Free cells after flood-fill: {np.sum(canvas > 127)}')

# Step 4: Erode inward to create margin from walls for safer racing line
safety_kernel = np.ones((11, 11), np.uint8)
track = cv2.erode(canvas, safety_kernel, iterations=1)
print(f'Free cells after safety erosion: {np.sum(track > 127)}')

# Visualize what we have
cv2.imwrite(OUT_PNG, track)
print(f'Saved track map: {OUT_PNG}')

# Save corresponding YAML (same origin/resolution as levine.yaml)
yaml_content = """image: levine_track.png
resolution: 0.050000
origin: [-51.224998, -51.224998, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(OUT_YAML, 'w') as f:
    f.write(yaml_content)
print(f'Saved YAML: {OUT_YAML}')

# Visualise the result
vis = cv2.cvtColor(track, cv2.COLOR_GRAY2BGR)
cv2.imwrite(os.path.join(MAPS_DIR, 'levine_track_debug.png'), vis)
print('Done! Now run the racing line generator on levine_track.yaml')
