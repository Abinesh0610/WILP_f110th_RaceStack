"""
racing_line_generator.py
========================
Generates the optimal racing line (centerline with speed profile) from a
SLAM-built map and publishes it as a nav_msgs/Path for the MPPI controller.

Purpose:
    After autonomous or manual mapping is complete, this node processes the
    saved occupancy grid map to extract a smooth centerline, computes a
    curvature-aware speed profile, saves the result as a CSV, and publishes
    it on /racing_line.

Pipeline:
    1. Load the .pgm / .yaml map from the maps/ directory.
    2. Convert OccupancyGrid to binary (free vs occupied).
    3. Compute the Euclidean Distance Transform (EDT) of the free space.
       The ridge of the EDT naturally follows the centerline of the track.
    4. Extract the ridge using morphological skeletonisation.
    5. Order the skeleton points into a continuous path.
    6. Smooth the path with a moving average filter.
    7. Compute curvature at each point and derive a speed profile:
           v_target = v_max * (1 - curvature_gain * curvature)
    8. Save the racing line as: maps/racing_line.csv (x, y, heading, target_speed)
    9. Publish as nav_msgs/Path on /racing_line.

Dependencies:
    scipy    — distance_transform_edt
    skimage  — morphology.skeletonize (scikit-image)
    cv2      — optional erosion / dilation preprocessing
"""

from __future__ import annotations

import csv
import math
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.ndimage import distance_transform_edt

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

try:
    from skimage.morphology import skeletonize
    SKIMAGE_AVAILABLE = True
except ImportError:
    SKIMAGE_AVAILABLE = False


class RacingLineGenerator(Node):
    """
    ROS 2 node that generates the optimal racing centerline from a SLAM map.

    Publishes:
        /racing_line (nav_msgs/Path) — reference path for MPPI controller

    Parameters (all live-tunable):
        map_path         : Absolute path to the .yaml map file
        v_max            : Maximum target speed [m/s]
        curvature_gain   : How much to reduce speed at curves
        smoothing_window : Moving average window size for path smoothing
        output_csv       : Path to save racing_line.csv
    """

    def __init__(self) -> None:
        """Initialise node, declare parameters, load map, generate and publish line."""
        super().__init__('racing_line_generator')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter('map_path', '')
        self.declare_parameter('v_max', 6.0)
        self.declare_parameter('v_min', 1.0)
        self.declare_parameter('curvature_gain', 3.0)
        self.declare_parameter('smoothing_window', 15)
        self.declare_parameter('output_csv', '')
        self.declare_parameter('publish_rate_hz', 1.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self._path_pub = self.create_publisher(Path, '/racing_line', 10)

        # Racing line data (set after generation)
        self._racing_line_path: Optional[Path] = None

        # ------------------------------------------------------------------
        # Generate the racing line from the map
        # ------------------------------------------------------------------
        if self.map_path:
            self._generate_and_publish()
        else:
            self.get_logger().warn(
                'map_path parameter is empty. '
                'Set it via: ros2 param set /racing_line_generator map_path <path>'
            )

        # ------------------------------------------------------------------
        # Timer: re-publish the racing line periodically so late subscribers
        # (like the MPPI controller) can receive it.
        # ------------------------------------------------------------------
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self._pub_timer = self.create_timer(period, self._publish_racing_line)

        self.get_logger().info(
            f'RacingLineGenerator ready | v_max={self.v_max} | '
            f'curvature_gain={self.curvature_gain}'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Load all parameters into instance variables."""
        self.map_path: str = self.get_parameter('map_path').value
        self.v_max: float = self.get_parameter('v_max').value
        self.v_min: float = self.get_parameter('v_min').value
        self.curvature_gain: float = self.get_parameter('curvature_gain').value
        self.smoothing_window: int = int(self.get_parameter('smoothing_window').value)
        self.output_csv: str = self.get_parameter('output_csv').value
        self.publish_rate_hz: float = self.get_parameter('publish_rate_hz').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Regenerate racing line if map_path or tuning params change."""
        self._load_params()
        if self.map_path:
            self._generate_and_publish()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_map(self) -> Tuple[Optional[np.ndarray], float, Tuple[float, float]]:
        """
        Load a SLAM Toolbox .yaml + .pgm map into a binary NumPy array.

        Returns
        -------
        binary_map : np.ndarray or None
            2D array: 1 = free space, 0 = occupied/unknown.
        resolution : float
            Map resolution in metres per cell.
        origin : tuple (x, y)
            World coordinates of the map's bottom-left corner.
        """
        yaml_path = self.map_path
        if not yaml_path.endswith('.yaml'):
            yaml_path = yaml_path + '.yaml'

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f'Map YAML not found: {yaml_path}')
            return None, 0.05, (0.0, 0.0)

        # --- Parse the YAML map descriptor ---
        import yaml  # Standard library safe import
        with open(yaml_path, 'r') as f:
            map_meta = yaml.safe_load(f)

        resolution: float = float(map_meta.get('resolution', 0.05))
        origin: List[float] = map_meta.get('origin', [0.0, 0.0, 0.0])
        pgm_path: str = os.path.join(
            os.path.dirname(yaml_path),
            map_meta.get('image', 'map.pgm')
        )

        # --- Load the .pgm image ---
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Could not load map image: {pgm_path}')
            return None, resolution, (origin[0], origin[1])

        # ROS map convention: 254=free (white), 205=unknown (grey), 0=occupied (black)
        # IMPORTANT: use a HIGH threshold so only truly-free (white) pixels pass.
        # free_thresh in YAML is a probability (0.196), which maps to pixel value ~50.
        # But that is the LOWER bound for free in probability space.
        # In the PGM: pixel=254 means p_free=1.0, pixel=0 means p_occ=1.0.
        # We want pixel > 200 (i.e. clearly free), NOT "> 50" which includes grey/unknown.
        binary_map = (img > 200).astype(np.uint8)

        # Erode free space by ~5 cells (0.25 m) to keep the skeleton away from walls
        erode_kernel = np.ones((9, 9), np.uint8)
        binary_map = cv2.erode(binary_map, erode_kernel, iterations=1)

        self.get_logger().info(
            f'Map loaded: {img.shape} | resolution={resolution} m | '
            f'free cells: {np.sum(binary_map)}'
        )
        return binary_map, resolution, (float(origin[0]), float(origin[1]))

    # ------------------------------------------------------------------
    # Racing line generation
    # ------------------------------------------------------------------

    def _generate_and_publish(self) -> None:
        """
        Full pipeline: load map → extract centerline → speed profile →
        save CSV → build Path message.
        """
        binary_map, resolution, origin = self._load_map()
        if binary_map is None:
            return

        # --- Step 1: Euclidean Distance Transform ---
        # EDT value at each free cell = distance to nearest occupied cell.
        # The ridge (local maxima) of this field lies on the track centerline.
        edt = distance_transform_edt(binary_map)

        # --- Step 2: Extract skeleton (ridge of EDT) ---
        if SKIMAGE_AVAILABLE:
            # skimage's Lee algorithm gives a clean 1-pixel-wide skeleton
            skeleton = skeletonize(binary_map.astype(bool)).astype(np.uint8)
        else:
            # Fallback: threshold the EDT at 30% of its max value
            edt_threshold = 0.3 * float(np.max(edt))
            skeleton = (edt > edt_threshold).astype(np.uint8)
            # Thin with morphological erosion to approximate skeleton
            kernel = np.ones((3, 3), np.uint8)
            skeleton = cv2.morphologyEx(skeleton, cv2.MORPH_ERODE, kernel)

        # --- Step 3: Extract ordered skeleton points ---
        waypoints = self._order_skeleton(skeleton, edt)
        if len(waypoints) < 5:
            self.get_logger().error('Skeleton too sparse — check the map quality.')
            return

        # --- Step 4: Convert pixel coordinates to world coordinates ---
        # Pixel (col, row) → world (x, y)
        # Note: row 0 = top of image in pixel space but bottom of world (flip Y)
        height = binary_map.shape[0]
        world_pts = np.array([
            [
                origin[0] + col * resolution,
                origin[1] + (height - row) * resolution
            ]
            for row, col in waypoints
        ], dtype=np.float64)

        # --- Step 5: Smooth the path ---
        world_pts = self._smooth_path(world_pts, self.smoothing_window)

        # --- Step 6: Compute headings ---
        diffs = np.diff(world_pts, axis=0)
        headings = np.arctan2(diffs[:, 1], diffs[:, 0])
        headings = np.append(headings, headings[-1])

        # --- Step 7: Compute curvature and speed profile ---
        curvatures = self._compute_curvature(world_pts)
        speeds = self.v_max * np.maximum(
            self.v_min / self.v_max,
            1.0 - self.curvature_gain * curvatures
        )
        speeds = np.clip(speeds, self.v_min, self.v_max)

        # --- Step 8: Save CSV ---
        csv_path = self.output_csv or os.path.join(
            os.path.dirname(self.map_path), 'racing_line.csv'
        )
        self._save_csv(csv_path, world_pts, headings, speeds)

        # --- Step 9: Build and store nav_msgs/Path ---
        self._racing_line_path = self._build_path_msg(world_pts, headings)
        self.get_logger().info(
            f'Racing line generated: {len(world_pts)} waypoints | '
            f'saved to {csv_path}'
        )

    def _order_skeleton(
        self,
        skeleton: np.ndarray,
        edt: np.ndarray
    ) -> List[Tuple[int, int]]:
        """
        Order the skeleton pixels into a continuous path by nearest-neighbour
        traversal, starting from the point with the highest EDT value (the
        widest part of the track — usually the main straight).

        Parameters
        ----------
        skeleton : np.ndarray
            Binary image with 1-pixel-wide skeleton.
        edt : np.ndarray
            Euclidean distance transform of the free space.

        Returns
        -------
        List[Tuple[int, int]]
            Ordered list of (row, col) pixel coordinates.
        """
        # Get all skeleton pixel coordinates
        rows, cols = np.where(skeleton > 0)
        if len(rows) == 0:
            return []

        all_pts = list(zip(rows.tolist(), cols.tolist()))

        # Start from the widest point (max EDT on skeleton)
        skeleton_edt = edt[rows, cols]
        start_idx = int(np.argmax(skeleton_edt))
        start = all_pts[start_idx]

        # Nearest-neighbour traversal
        visited = {start}
        ordered = [start]
        remaining = set(all_pts) - visited

        current = start
        while remaining:
            cr, cc = current
            # Find nearest unvisited point using Manhattan distance for speed
            best_dist = float('inf')
            best_pt = None
            for pr, pc in remaining:
                d = abs(pr - cr) + abs(pc - cc)
                if d < best_dist:
                    best_dist = d
                    best_pt = (pr, pc)
                    if d == 1:
                        break  # Can't get closer in Manhattan distance
            if best_pt is None or best_dist > 10:
                # Gap too large → break (disconnected skeleton)
                break
            ordered.append(best_pt)
            visited.add(best_pt)
            remaining.discard(best_pt)
            current = best_pt

        return ordered

    def _smooth_path(self, pts: np.ndarray, window: int) -> np.ndarray:
        """
        Smooth a 2D path using a moving average filter.

        Parameters
        ----------
        pts : np.ndarray, shape (N, 2)
            Raw waypoints [x, y].
        window : int
            Size of the moving average window.

        Returns
        -------
        np.ndarray, shape (N, 2)
            Smoothed waypoints.
        """
        if window < 2 or len(pts) < window:
            return pts
        kernel = np.ones(window) / window
        smoothed_x = np.convolve(pts[:, 0], kernel, mode='same')
        smoothed_y = np.convolve(pts[:, 1], kernel, mode='same')
        return np.column_stack([smoothed_x, smoothed_y])

    def _compute_curvature(self, pts: np.ndarray) -> np.ndarray:
        """
        Compute the discrete curvature at each waypoint using the
        cross-product formula for three consecutive points.

        κ = 2 * ||(p_{i-1} - p_i) × (p_{i+1} - p_i)|| /
                 (||p_{i-1} - p_i|| * ||p_{i+1} - p_i|| * ||p_{i-1} - p_{i+1}||)

        Parameters
        ----------
        pts : np.ndarray, shape (N, 2)

        Returns
        -------
        np.ndarray, shape (N,)
            Curvature at each waypoint (positive, normalised).
        """
        N = len(pts)
        curvature = np.zeros(N)

        for i in range(1, N - 1):
            a = pts[i - 1]
            b = pts[i]
            c = pts[i + 1]
            ba = a - b
            bc = c - b
            # 2D cross product magnitude
            cross = abs(ba[0] * bc[1] - ba[1] * bc[0])
            denom = (np.linalg.norm(ba) * np.linalg.norm(bc)
                     * np.linalg.norm(a - c) + 1e-9)
            curvature[i] = 2.0 * cross / denom

        # Fill endpoints with neighbours
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]

        return curvature

    def _save_csv(
        self,
        path: str,
        pts: np.ndarray,
        headings: np.ndarray,
        speeds: np.ndarray
    ) -> None:
        """
        Save the racing line as a CSV file: x, y, heading, target_speed.

        Parameters
        ----------
        path : str
            Output file path.
        pts : np.ndarray, shape (N, 2)
        headings : np.ndarray, shape (N,)
        speeds : np.ndarray, shape (N,)
        """
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'heading', 'target_speed'])
            for i in range(len(pts)):
                writer.writerow([
                    f'{pts[i, 0]:.6f}',
                    f'{pts[i, 1]:.6f}',
                    f'{headings[i]:.6f}',
                    f'{speeds[i]:.4f}'
                ])
        self.get_logger().info(f'Racing line CSV saved: {path}')

    def _build_path_msg(self, pts: np.ndarray, headings: np.ndarray) -> Path:
        """
        Build a nav_msgs/Path message from waypoints and headings.

        Heading is encoded in the pose quaternion (z-rotation only).
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i in range(len(pts)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pts[i, 0])
            pose.pose.position.y = float(pts[i, 1])
            pose.pose.position.z = 0.0
            # Encode heading as quaternion (rotation around Z)
            half_yaw = headings[i] / 2.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            path_msg.poses.append(pose)

        return path_msg

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_racing_line(self) -> None:
        """Periodically re-publish the stored racing line path."""
        if self._racing_line_path is not None:
            # Update timestamp
            self._racing_line_path.header.stamp = self.get_clock().now().to_msg()
            self._path_pub.publish(self._racing_line_path)


def main(args=None) -> None:
    """Entry point for the racing_line_generator ROS 2 node."""
    rclpy.init(args=args)
    node = RacingLineGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RacingLineGenerator shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
