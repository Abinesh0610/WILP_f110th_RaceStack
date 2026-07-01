# F1TENTH Race Stack by WILP
<img width="1600" height="739" alt="hero-bg" src="https://github.com/user-attachments/assets/f14fda60-5256-4c28-adec-1de83a03e698" />

> A production-quality, ROS 2 Humble autonomous racing stack for the F1TENTH Traxxas Slash 4x4 platform. This stack handles everything from system identification and map generation to advanced Time Trial and Head-to-Head racing algorithms, complete with a hardware deadman switch for safety.

---

## 1. Hardware Bill of Materials (BOM)

| # | Component | Specification | Notes |
|---|-----------|--------------|-------|
| 1 | **Chassis** | Traxxas Slash 4x4 | 1:10 scale, rear/4WD |
| 2 | **Wheelbase** | 324 mm | Front–rear axle centre-to-centre |
| 3 | **Width & Length**| 296 mm x 568 mm | Full chassis dimensions |
| 4 | **Mass** | ~3.0 kg | Car + electronics + battery |
| 5 | **Compute** | Intel NUC 12 Pro | i7-1260P, 12-core, CPU-only |
| 6 | **LiDAR** | Hokuyo UST-10LX | 270° FoV, 10 m range, 40 Hz |
| 7 | **Motor Driver**| VESC MK-VI | Brushless DC, ERPM-controlled |
| 8 | **Joystick** | RadioMaster MT12 | Connected via `/dev/input/js0` |

---

## 2. Dependencies & Installation

### ROS 2 System Dependencies
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-rqt-reconfigure \
  ros-humble-rviz2
```

### Python Dependencies
```bash
pip install numpy scipy opencv-python matplotlib scikit-image
```

### Building the Workspace
```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Go to your workspace
cd ~/racer_ws

# 3. Build the package
colcon build --symlink-install --packages-select f1tenth_race_stack

# 4. Build the workspace
colcon build --symlink-install --packages-select f1tenth_race_stack

# 5. Source the workspace
source install/setup.bash
```

---

## 3. Quick Reference Commands (Real Car Hardware)

Here are the exact terminal commands to launch each mode of the race stack. 

*(**Important Safety Rule**: For every command below, you must HOLD Button 5 (SF) on your RadioMaster MT12 joystick for the car to move. Release it to emergency stop).*

### Mapping (Create your track map)
**Manual Joystick Mapping:**
```bash
ros2 launch f1tenth_race_stack mapping_joystick.launch.py
```
**Autonomous Mapping (Follow-The-Gap):**
```bash
ros2 launch f1tenth_race_stack mapping_autonomous.launch.py
```
**Save the Map (run in a new terminal after mapping):**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track
```

### Path Generation
**Generate optimal racing line from your saved map:**
```bash
ros2 run f1tenth_race_stack racing_line_generator \
  --ros-args -p map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml
```

### Racing
**Time Trials (Pure Pursuit - Fastest solo lap):**
```bash
ros2 launch f1tenth_race_stack time_trial.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv
```
**Head-to-Head (MPPI - Dynamic overtaking and obstacle avoidance):**
```bash
ros2 launch f1tenth_race_stack head_to_head.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv
```

---

## 4. For New Users: Running in Simulation

If you are new to the package or do not have physical F1TENTH hardware yet, you can test everything using the official 2D physics simulator! Our stack is **100% plug-and-play compatible** with `f1tenth_gym_ros`.

### Step 1: Install the Simulator
Run this once to download and build the simulator in your workspace:
```bash
# Install the core Python physics engine
pip3 install git+https://github.com/f1tenth/f1tenth_gym.git

# Clone the ROS 2 simulator wrapper
cd ~/racer_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

# Build it
cd ~/racer_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_gym_ros
```

### Step 2: Run the Simulation
We have provided dedicated `sim_` launch files that automatically bypass the hardware deadman switch and disable hardware nodes like the VESC bridge.

**Terminal 1 (Start the Simulator World):**
```bash
source ~/racer_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 (Start our AI Brain):**
```bash
source ~/racer_ws/install/setup.bash

# Option A: Run Time Trials (Pure Pursuit)
ros2 launch f1tenth_race_stack sim_time_trial.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv

# Option B: Run Head-to-Head (MPPI)
ros2 launch f1tenth_race_stack sim_head_to_head.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv
```

---

## 5. Node Architecture & Explanations

Here is a breakdown of every node in this stack and how it works:

### 🏎️ Controllers (The Brains)
* **`mppi_controller.py` (Model Predictive Path Integral)**
  * **Role:** Used for **Head-to-Head Racing**.
  * **How it works:** It uses the CPU to simulate 1,000 different future trajectories (rollouts) in parallel using a dynamic bicycle model. It evaluates each trajectory based on speed, distance to the racing line, and distance to obstacles (LiDAR points). It averages the best trajectories to find an optimal steering and speed command. Because it naturally avoids obstacles, it's perfect for overtaking opponents.
  * **vs ForzaETH:** While ForzaETH often relies on Game-Theoretic NMPC or Lattice Planners for overtaking (which can be very computationally heavy), our stack uses a highly CPU-optimized, vectorized NumPy implementation of MPPI. This allows our stack to achieve dynamic overtaking without needing a GPU, running comfortably on an Intel NUC.

* **`pure_pursuit.py` (Pure Pursuit)**
  * **Role:** Used for **Time Trials**.
  * **How it works:** A geometric tracker. It looks ahead a certain distance on the racing line, draws an arc from the car to that point, and calculates the required steering angle. It is extremely fast (runs at 40+ Hz), deterministic, and produces the smoothest, fastest laps when there are no opponents on track.
  * **vs ForzaETH:** ForzaETH typically uses a highly complex Local Trajectory Tracker or NMPC even for time trials. Our Pure Pursuit is a deliberate, lightweight alternative. Because our `racing_line_generator` perfectly calculates the speed profile beforehand, a simple geometric tracker is all that is needed to achieve near-optimal time trial laps with zero computational overhead.

### 🗺️ Mapping & Path Planning
* **`follow_the_gap.py` (FTG)**
  * **Role:** Used for **Autonomous Mapping**.
  * **How it works:** A purely reactive algorithm. It reads the LiDAR scan, creates a "safety bubble" around the closest obstacles, finds the largest gap of free space, and steers towards the furthest point in that gap. It scales its speed down during sharp turns.
  * **vs ForzaETH:** This aligns closely with standard F1TENTH reactive architectures. It provides a robust fail-safe and an easy way to explore unknown tracks before global optimization is possible.

* **`racing_line_generator.py`**
  * **Role:** Offline path generation and **Curvature-Aware Speed Profiling**.
  * **How it works:** Takes a saved SLAM map (`.yaml`), uses Euclidean Distance Transforms to find the track boundaries, extracts the centerline, smooths it, and computes an aggressive speed profile based on track curvature.
  * **vs ForzaETH (Sector Tuning):** ForzaETH often uses manual **"Sector Tuning,"** where developers must physically divide the track into sectors (e.g., straightaway, hairpin) and manually assign different target speeds and weights to each sector. **Our stack automates this.** The generator mathematically evaluates the curvature of every millimeter of the track. It automatically assigns high speeds (`v_max`) to straight lines and dynamically ramps down the target speed as a corner sharpens. Therefore, the controllers automatically brake for corners and accelerate on straights without any manual sector definitions.

### 🛡️ Safety & State Management
* **`deadman_switch.py` (Hardware Safety Gate)**
  * **Role:** The ultimate safety layer. 
  * **How it works:** Every autonomous planner publishes to `/drive_cmd`. The deadman switch subscribes to this and to the joystick. **Unless Button 5 (SF) is HELD DOWN**, it blocks `/drive_cmd` and instead blasts zero-velocity commands to the wheels at 50 Hz. This guarantees the car stops instantly if you let go of the controller.
* **`race_state_machine.py`**
  * **Role:** High-level coordinator.
  * **How it works:** Manages states (`IDLE`, `MAPPING_MANUAL`, `MAPPING_AUTO`, `TIME_TRIAL`, `HEAD_TO_HEAD`).

### ⚙️ Utilities
* **`vesc_bridge.py`**: Converts Ackermann commands (`speed`, `steering`) into VESC ERPM and Servo signals.
* **`system_id_driver.py`**: An open-loop controller used strictly for calibrating the ERPM-to-m/s ratio.
* **`visualizer.py`**: Publishes rich ROS 2 `MarkerArray` topics to draw the racing line, velocity arrows, and safety bubbles in RViz.

---

## 6. Workflow: From Scratch to Advanced Racing

Follow this exact sequence to deploy the car on a new track.

### Phase 1: System Identification (First-Time Only)
Calibrate how the motor's RPM relates to real-world speed.
1. Place the car on a 10-meter straight.
2. Launch the ID node:
   ```bash
   ros2 launch f1tenth_race_stack system_id.launch.py
   ```
3. Look at the terminal output to find the `ERPM` for a given speed (e.g., 2.0 m/s). 
4. Update `erpm_gain` in `config/vesc_bridge_params.yaml`.

### Phase 2: Build a Track Map
Create a 2D occupancy grid of the track using SLAM.

**Option A: Manual Driving (Recommended)**
```bash
ros2 launch f1tenth_race_stack mapping_joystick.launch.py
```
* Hold **Button 5 (SF)** to enable the deadman switch.
* Drive the car around the track using the left (throttle) and right (steering) sticks.
* Drive 2-3 laps to close the loop.

**Option B: Autonomous Driving**
```bash
ros2 launch f1tenth_race_stack mapping_autonomous.launch.py
```
* Hold **Button 5 (SF)**. The Follow-The-Gap node will autonomously navigate the track.

**Save the Map:** (Run in a new terminal)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track
```

### Phase 3: Generate the Racing Line
Generate the optimal path and speed profile from the map.
```bash
ros2 run f1tenth_race_stack racing_line_generator \
  --ros-args \
  -p map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  -p v_max:=8.0 \
  -p curvature_gain:=2.5
```
This saves `racing_line.csv` in the `maps/` folder.

### Phase 4: Time Trials (Solo Racing)
For the absolute fastest lap time on an empty track, use Pure Pursuit.
```bash
ros2 launch f1tenth_race_stack time_trial.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv
```
* Hold **Button 5 (SF)** to launch the car. 
* *Tip: Tune `lookahead_distance` if the car is oscillating.*

### Phase 5: Head-to-Head (Racing with Opponents)
When racing against other cars, use MPPI for dynamic overtaking.
```bash
ros2 launch f1tenth_race_stack head_to_head.launch.py \
  map_path:=$HOME/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track.yaml \
  racing_line_csv:=$HOME/racer_ws/src/f1tenth_race_stack/maps/racing_line.csv
```
* Hold **Button 5 (SF)** to launch. The car will swerve around obstacles.

---

## 7. Dynamic Parameter Tuning (Live Tuning)

Every single parameter in this stack is **live-tunable**. You do not need to restart the nodes to change behavior!

When you launch any racing mode, a GUI called `rqt_reconfigure` will open automatically. You can use sliders to adjust parameters in real-time.

Alternatively, use the CLI in a new terminal window while the car is running:

### Tuning Mapping Speeds (Follow-The-Gap & Manual)
```bash
# Increase the top speed of the autonomous FTG mapping
ros2 param set /follow_the_gap ftg.max_speed 4.0

# Change how much FTG slows down for sharp corners
ros2 param set /follow_the_gap ftg.min_speed 1.0

# Increase the maximum allowed speed when driving manually with the joystick
ros2 param set /deadman_switch manual_max_speed 5.0
```

### Tuning Time Trials (Pure Pursuit)
```bash
# Make the car look further ahead (smoothes out wobbly steering)
ros2 param set /pure_pursuit pure_pursuit.lookahead_distance 1.5

# Scale the speed to 80% of max for a safe warmup lap
ros2 param set /pure_pursuit pure_pursuit.speed_scale 0.8
```

### Tuning Head-to-Head (MPPI)
```bash
# Make the car more afraid of opponents (steers wider around them)
ros2 param set /mppi_controller mppi.weight_obstacle_penalty 1000.0

# Force the car to stick tighter to the optimal racing line
ros2 param set /mppi_controller mppi.weight_reference_track 20.0

# Increase computational rollouts (Better decisions, higher CPU load)
ros2 param set /mppi_controller mppi.K 1500
```

---

## 6. Parameter Reference Tables

### `pure_pursuit_params.yaml`
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | 1.0 m | Base distance to aim for on the path. |
| `lookahead_gain` | 0.2 | Increases lookahead proportionally with speed. |
| `speed_scale` | 1.0 | Multiplier for the racing line target speed. |

### `mppi_params.yaml`
| Parameter | Default | Description |
|-----------|---------|-------------|
| `mppi.K` | 1000 | Number of parallel rollouts simulated. |
| `mppi.T` | 20 | Prediction horizon (number of steps). |
| `mppi.weight_reference_track` | 10.0 | Penalty for drifting off the racing line. |
| `mppi.weight_obstacle_penalty` | 500.0 | Penalty for getting too close to LiDAR points. |
| `mppi.obstacle_clearance_m` | 0.35 m | The safety radius around the car. |

### `deadman_switch_params.yaml`
| Parameter | Default | Description |
|-----------|---------|-------------|
| `deadman_button_idx` | 5 | Joystick button index (RadioMaster SF switch). |
| `manual_max_speed` | 3.0 m/s | Top speed allowed during manual mapping. |
| `joy_timeout_s` | 0.5 s | Stops car if controller disconnects. |
