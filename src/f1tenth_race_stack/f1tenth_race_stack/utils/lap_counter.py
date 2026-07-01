#!/usr/bin/env python3
"""
lap_counter.py
==============
A ROS 2 node that detects completed laps during a Time Trial session
and displays the count in a live popup window with lap timing.

Algorithm:
  1. Record the car's starting position (x, y) from the first odometry message.
  2. Wait until the car has 'departed' from the start (dist > departure_threshold).
  3. When the car returns within arrival_threshold of the start → increment lap count.
  4. Repeat. Everything resets to 0 each time this node is (re)started.

GUI:
  - Always-on-top window with a large green lap counter.
  - Shows last lap time and best lap time in real-time.

ROS 2 topics consumed:
  /ego_racecar/odom (nav_msgs/Odometry)
"""

import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from typing import List, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LapCounter(Node):
    """Counts completed laps and shows a live GUI popup."""

    def __init__(self) -> None:
        super().__init__('lap_counter')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('departure_threshold', 2.5)   # [m] must travel this far from start
        self.declare_parameter('arrival_threshold', 1.5)     # [m] this close to start = lap complete
        self.declare_parameter('odom_topic', '/ego_racecar/odom')

        self._departure_threshold: float = self.get_parameter('departure_threshold').value
        self._arrival_threshold: float   = self.get_parameter('arrival_threshold').value
        odom_topic: str                  = self.get_parameter('odom_topic').value

        # ── State ──────────────────────────────────────────────────────────────
        self._start_x: Optional[float] = None
        self._start_y: Optional[float] = None
        self._has_departed: bool       = False
        self._lap_count: int           = 0
        self._lap_times: List[float]   = []
        self._lap_start_wall: float    = 0.0   # wall-clock time of last lap start

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.get_logger().info(
            f'LapCounter ready | departure={self._departure_threshold} m | '
            f'arrival={self._arrival_threshold} m | topic={odom_topic}')

        # ── GUI in background thread ───────────────────────────────────────────
        self._lap_var:  Optional[tk.StringVar] = None
        self._last_var: Optional[tk.StringVar] = None
        self._best_var: Optional[tk.StringVar] = None
        threading.Thread(target=self._run_gui, daemon=True).start()

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # First message — record start position
        if self._start_x is None:
            self._start_x = x
            self._start_y = y
            self._lap_start_wall = time.monotonic()
            self.get_logger().info(
                f'Start/Finish line recorded at ({x:.2f}, {y:.2f})')
            return

        dist = ((x - self._start_x) ** 2 + (y - self._start_y) ** 2) ** 0.5

        if not self._has_departed:
            # Phase 1 — wait for car to leave the start zone
            if dist > self._departure_threshold:
                self._has_departed = True
        else:
            # Phase 2 — wait for car to return to start zone
            if dist < self._arrival_threshold:
                self._has_departed = False
                self._lap_count += 1

                # Compute elapsed lap time using wall clock
                now = time.monotonic()
                lap_time = now - self._lap_start_wall
                self._lap_times.append(lap_time)
                self._lap_start_wall = now

                best = min(self._lap_times)
                self.get_logger().info(
                    f'LAP {self._lap_count} | '
                    f'Time: {lap_time:.2f}s | Best: {best:.2f}s')

                self._update_gui(lap_time, best)

    # ------------------------------------------------------------------
    # GUI update (called from ROS spin thread — thread-safe via StringVar)
    # ------------------------------------------------------------------
    def _update_gui(self, last_time: float, best_time: float) -> None:
        if self._lap_var is not None:
            self._lap_var.set(str(self._lap_count))
        if self._last_var is not None:
            self._last_var.set(f'Last:  {last_time:.2f} s')
        if self._best_var is not None:
            self._best_var.set(f'Best:  {best_time:.2f} s')

    # ------------------------------------------------------------------
    # Tkinter GUI
    # ------------------------------------------------------------------
    def _run_gui(self) -> None:
        """Build and run the lap counter popup in a background thread."""
        root = tk.Tk()
        root.title('F1TENTH — Lap Counter')
        root.configure(bg='#0d0d0d')
        root.geometry('420x320')
        root.resizable(False, False)
        root.attributes('-topmost', True)   # always on top

        # ── Fonts ─────────────────────────────────────────────────────
        title_font = tkfont.Font(family='Helvetica', size=13, weight='bold')
        count_font = tkfont.Font(family='Helvetica', size=96, weight='bold')
        stat_font  = tkfont.Font(family='Helvetica', size=16, weight='bold')

        # ── Header ────────────────────────────────────────────────────
        tk.Label(root, text='LAPS COMPLETED',
                 font=title_font, bg='#0d0d0d', fg='#666666').pack(pady=(18, 0))

        # ── Big lap counter ───────────────────────────────────────────
        self._lap_var = tk.StringVar(value='0')
        tk.Label(root, textvariable=self._lap_var,
                 font=count_font, bg='#0d0d0d', fg='#00ff88').pack(pady=(0, 4))

        # ── Lap time stats ────────────────────────────────────────────
        self._last_var = tk.StringVar(value='Last:  --')
        self._best_var = tk.StringVar(value='Best:  --')

        tk.Label(root, textvariable=self._last_var,
                 font=stat_font, bg='#0d0d0d', fg='#ffffff').pack()
        tk.Label(root, textvariable=self._best_var,
                 font=stat_font, bg='#0d0d0d', fg='#ffd700').pack()

        # ── Footer hint ───────────────────────────────────────────────
        tk.Label(root, text='Ctrl+C in terminal to stop',
                 font=tkfont.Font(family='Helvetica', size=9),
                 bg='#0d0d0d', fg='#333333').pack(side='bottom', pady=8)

        root.mainloop()


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = LapCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
