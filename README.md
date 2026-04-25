# Autonomous Maze-Solving TurtleBot3 — ROS 2 Gazebo Simulation

A **ROS 2 Humble** simulation of a **TurtleBot3 Waffle** navigating a **procedurally generated maze** that is different every run. The robot computes the optimal BFS path, executes it with a closed-loop waypoint controller, and uses Nav2 as a fallback — all without any human guidance.

Supports two launch modes:
- **`use_slam:=false`** *(default / recommended)* — uses a pre-generated occupancy map derived directly from the maze grid.
- **`use_slam:=true`** *(experimental)* — uses `slam_toolbox` for live online mapping while navigating.

Both modes are wired up and launched from the same launch file.

---

## How it Works (Plain English)

1. **Every time you launch**, a fresh random maze is generated (different seed → different layout).
2. The maze spawner writes out:
   - A **Gazebo SDF model** (walls + start/goal markers) — spawned live into the simulator.
   - A **2D occupancy map** (PGM + YAML) — loaded by Nav2's map server.
   - A **runtime JSON payload** — consumed by the solver (contains the grid, start/goal cells, and world coordinates).
3. You then run the **maze solver**, which:
   - Reads the runtime payload.
   - Runs BFS to find the shortest path through the grid.
   - Drives the robot waypoint-by-waypoint using `/odom` feedback (`use_slam:=false`) or map-frame TF (`use_slam:=true`).
   - Falls back to Nav2 segmented goals if direct driving gets stuck.
   - Falls back to direct driving again if Nav2 fails.

---

## Repository Structure

```
Maze-Solving-Robot/
├── src/
│   └── autonomous_tb3/
│       ├── launch/
│       │   ├── tb3_maze_navigation.launch.py   # Main launch file (SLAM + no-SLAM)
│       │   ├── navigation.launch.py            # Legacy static-world nav flow
│       │   └── mapping.launch.py               # Cartographer-based mapping (alt)
│       ├── script/
│       │   ├── maze_generator.py               # Recursive-backtracker maze algorithm
│       │   ├── procedural_world_spawner.py     # Generates maze, spawns it, writes runtime files
│       │   ├── maze_solver.py                  # BFS path + waypoint controller + Nav2 fallback
│       │   ├── occupancy_grid_pub.py           # Occupancy grid publisher utility
│       │   └── entity_spawner.py               # Generic Gazebo spawn client helper
│       ├── config/
│       │   ├── tb3_nav_params.yaml             # Nav2 tuning (speeds, inflation, recovery)
│       │   ├── tb3_nav.rviz                    # RViz2 config for navigation view
│       │   ├── maze_map.pgm / .yaml            # Static reference map (legacy)
│       │   ├── tb3_world_2d_map.pgm / .yaml    # Pre-built world map (legacy)
│       │   └── turtlebot3_cartographer.lua     # Cartographer SLAM config
│       ├── CMakeLists.txt
│       └── package.xml
├── README.md
├── PROJECT_CONTEXT.md   # Code flow cheat sheet / quick reference
├── reqs.txt             # apt dependencies
└── Thumbnail.png
```

---

## Runtime Files (Written Fresh Each Run)

These are created in `/tmp/autonomous_tb3/` by the spawner node on every launch:

| File | Purpose |
|---|---|
| `maze_runtime.json` | Grid, seed, start/goal cells + world poses, resolution, origin |
| `maze_runtime_map.pgm` | Occupancy map image (0 = wall, 254 = free) for Nav2 map_server |
| `maze_runtime_map.yaml` | Map metadata pointing to PGM; loaded by Nav2 |

Because these are regenerated each run, **every run has a unique maze**.

---

## Requirements

- Ubuntu 22.04 with **ROS 2 Humble** installed and the ROS apt repository configured.
- Gazebo Classic (`gazebo11`)
- `turtlebot3_gazebo` simulation package
- `navigation2` / `nav2_bringup`
- `slam_toolbox` (required even for `use_slam:=false` because it is a launch dependency)
- `python3-colcon-ros` (needed so `colcon build` picks up Python packages)

---

## Installation & Build

```bash
# 1. Clone the repository
git clone https://github.com/bugsNburgers/Maze-Solving-Robot.git
cd Maze-Solving-Robot/

# 2. Install system dependencies
sudo apt-get update
grep -v '^#' reqs.txt | xargs sudo apt-get install -y

# 3. Make Python scripts executable (required for ament_cmake packages)
chmod +x src/autonomous_tb3/script/*.py

# 4. Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> **Note:** If `colcon build` finishes with `0 packages`, you are missing `python3-colcon-ros` — it is listed in `reqs.txt`.

---

## Running the Simulation

### Mode 1 — No-SLAM

Uses a pre-generated map from the maze grid. AMCL provides localization. Direct odometry control is used by the solver.

**Terminal 1 — Launch Gazebo + Nav2:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py use_slam:=false
```

**Terminal 2 — Run the maze solver (keep Terminal 1 running):**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomous_tb3 maze_solver.py
```

---

### Mode 2 — SLAM Mode

Uses `slam_toolbox` for online mapping instead of the pre-generated map. The solver detects SLAM automatically and adjusts its coordinate alignment. Nav2 is preferred in this mode, with direct-drive fallback.

**Terminal 1 — Launch Gazebo + SLAM:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py use_slam:=true
```

**Terminal 2 — Run the maze solver:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomous_tb3 maze_solver.py
```

> **Note:** SLAM mode can be unstable in some environments (slam_toolbox lifecycle activation failures, TF delays). If the solver reports `map→base_link TF unavailable`, wait a few seconds for SLAM to stabilize, or prefer `use_slam:=false`.

---

## What Happens After Launch

Both windows — **Gazebo Classic** and **RViz2** — open automatically.

- In **Gazebo**: you see the robot and the generated maze walls. A **green sphere** marks the start and a **red sphere** marks the goal.
- In **RViz2**: you see the 2D map with the robot's pose estimate updating in real time.

Once the solver starts, expect to see:
```
Nav2 is ready for use!
Computed shortest path with N steps; starting direct route execution
Navigating to goal ...
Reached destination across all path segments
```

### Navigation Strategy

The solver uses a layered strategy:

```
1. Direct waypoint controller (primary)
   └── Drives cell-to-cell using /odom (or map TF in SLAM mode)
   └── Unstick maneuver on stagnation
   └── Path re-anchor after each failure

2. Nav2 segmented goals (first fallback)
   └── If direct drive fails repeatedly
   └── Path subsampled into stride-2 segments

3. Direct controller again (second fallback)
   └── If Nav2 segments fail
```

In SLAM mode, the order is: Nav2 global goal → Nav2 segmented → direct controller.

---

## Operator Controls

These ROS 2 service calls work at any time while the solver is running:

| Action | Command |
|---|---|
| Emergency stop (immediate) | `ros2 service call /maze_solver/emergency_stop std_srvs/srv/SetBool "{data: true}"` |
| Release emergency stop | `ros2 service call /maze_solver/emergency_stop std_srvs/srv/SetBool "{data: false}"` |
| Pause (safe stop) | `ros2 service call /maze_solver/pause std_srvs/srv/SetBool "{data: true}"` |
| Resume | `ros2 service call /maze_solver/pause std_srvs/srv/SetBool "{data: false}"` |
| Status query | `ros2 service call /maze_solver/status std_srvs/srv/Trigger "{}"` |

---

## Tuning Parameters

### Goal distance (how far the goal is from start)
Edit `procedural_world_spawner.py` → `_reachable_short_hop_goal(...)`:
```python
min_steps=35,   # minimum BFS steps from start to goal
max_steps=60,   # maximum BFS steps from start to goal
```

### Robot speed (direct-drive mode)
At runtime (no rebuild needed):
```bash
ros2 param set /maze_solver direct_max_linear_speed 0.10
ros2 param set /maze_solver direct_min_linear_speed 0.04
ros2 param set /maze_solver direct_max_angular_speed 0.80
```
Or edit `maze_solver.py` → `drive_to_waypoint()` and `drive_forward_distance()`.

### Robot speed (Nav2 controller)
Edit `config/tb3_nav_params.yaml`:
```yaml
controller_server:
  FollowPath:
    desired_linear_vel: 0.10
```

### Maze size
Edit `maze_generator.py`:
```python
MAZE_HEIGHT = 21   # must be odd
MAZE_WIDTH  = 21   # must be odd
```
Also update `x_pose` / `y_pose` defaults in `tb3_maze_navigation.launch.py` to place the robot at the new entry cell.

---

## Key Topics

| Topic | Direction | Description |
|---|---|---|
| `/scan` | → Nav2 | Simulated LiDAR |
| `/odom` | → solver, Nav2 | Wheel odometry |
| `/tf`, `/tf_static` | broadcast | Robot + map frames |
| `/map` | → RViz2, Nav2 | Occupancy grid from map_server |
| `/amcl_pose` | → solver | AMCL localization estimate |
| `/cmd_vel` | solver / Nav2 → robot | Velocity commands |
| `/initialpose` | solver → AMCL | Seeds AMCL at start |
| `/procedural_maze/start` | spawner → solver | World pose of start cell |
| `/procedural_maze/goal` | spawner → solver | World pose of goal cell |
| `/navigate_to_pose` | solver → Nav2 | Nav2 action goal |

> **Warning:** Do not send manual "2D Nav Goal" from RViz2 while the solver is in direct-drive mode — this creates conflicting `/cmd_vel` sources.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Entity [waffle] already exists` | Kill stale processes: `pkill -f gzserver; pkill -f gzclient; pkill -f rviz2` |
| `amcl/get_state service not available` | Nav2 is not fully active; keep Terminal 1 running and wait |
| `map→base_link TF unavailable` (SLAM mode) | Wait a few seconds for slam_toolbox to start publishing TF |
| Solver says `Timed out waiting for runtime payload` | The spawner node crashed; check Terminal 1 logs |
| Robot spins in place repeatedly | Nav2 recovery is triggering; prefer `use_slam:=false` or tune inflation radius |
| `colcon build` gives 0 packages | Install `python3-colcon-ros` (in `reqs.txt`) |

---

## Software Stack

| Component | Version / Package |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Simulator | Gazebo Classic (gazebo11) |
| Robot model | TurtleBot3 Waffle (`turtlebot3_gazebo`) |
| Navigation | Navigation2 (`nav2_bringup`) |
| SLAM | slam_toolbox (`online_async_launch.py`) |
| Maze algorithm | Recursive backtracker (custom, `maze_generator.py`) |
| Path planning | BFS on 4-connected grid (custom, `maze_solver.py`) |
| Language | Python 3 |
| Build system | ament_cmake |

---

At the same time, the robot follows the same trajectory in the **2D maze map** in **RViz2**.

