## Maze Project Step-by-Step Prompt Pack

Use this file as your execution script with me. For each phase:
1. Paste the Phase Prompt in chat.
2. I execute and report.
3. You paste the next phase prompt.

---

## Phase 0 Prompt: Baseline Snapshot
Paste this in chat:
Please run Phase 0 baseline checks only, no code changes. Run the exact commands below and summarize results in a checklist.

Commands:
cd /home/prateek/Maze
source /opt/ros/humble/setup.bash
colcon list
ros2 pkg list | rg "autonomous_tb3|nav2_bringup|slam_toolbox|turtlebot3_gazebo"
rg -n "maze_map.yaml|maze_mapping_slam|ld.add_action\(navigation\)|spawn_turtlebot_cmd" src/autonomous_tb3/launch/tb3_maze_navigation.launch.py
rg -n "initial_pose.pose.position|goal_pose.pose.position|setInitialPose|goToPose" src/autonomous_tb3/script/maze_solver.py

Expected outcome:
- Confirm static map dependency exists.
- Confirm SLAM launch include exists but is not active in final actions.
- Confirm hardcoded start and goal in solver.

---

## Phase 1 Prompt: Build and Dependency Hardening
Paste this in chat:
Run Phase 1 setup and build hardening. Execute commands exactly, fix only environment issues, and report pass or fail for each command.

Commands:
cd /home/prateek/Maze
sudo apt-get update
grep -v '^#' reqs.txt | xargs sudo apt-get install -y
chmod +x src/autonomous_tb3/script/*.py
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py --show-args

Expected outcome:
- Clean build succeeds.
- Launch file parses without syntax/runtime import errors.

---

## Phase 2 Prompt: Current Runtime Validation
Paste this in chat:
Run Phase 2 runtime validation only. Start launch in one terminal context, probe nodes and topics from another, no code edits.

Commands:
Terminal A:
cd /home/prateek/Maze && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py

Terminal B:
cd /home/prateek/Maze && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 node list
ros2 topic list | rg "map|scan|odom|tf"
ros2 param list | rg "amcl|bt_navigator|controller_server"

Expected outcome:
- Gazebo, RViz, Nav2 stack come up.
- Baseline behavior recorded before procedural implementation.

---

## Phase 3 Prompt: Implement Procedural Maze Generator
Paste this in chat:
Implement Phase 3 only. Create a procedural maze generator script in the package scripts folder, integrate it for install, and do not modify unrelated files. After coding, build and run generator test.

Commands after implementation:
cd /home/prateek/Maze
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run autonomous_tb3 maze_generator.py

Expected outcome:
- Maze data generated each run.
- Start and goal coordinates produced with guaranteed solvable layout.
- Fixed grid size is enforced (21x21) unless explicitly changed in code.

---

## Phase 4 Prompt: Implement Procedural World Spawner
Paste this in chat:
Implement Phase 4 only. Add a procedural world spawner script that converts generated maze to spawnable model and spawns via /spawn_entity. Expose start and goal for solver consumption.

Commands after implementation:
cd /home/prateek/Maze
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 service list | rg spawn_entity
ros2 topic list | rg "procedural|maze|goal|start"
python3 -c "import json; d=json.load(open('/tmp/autonomous_tb3/maze_runtime.json')); print(d['height'], d['width'])"

Expected outcome:
- Fresh maze spawns in Gazebo.
- Dynamic endpoints are published/discoverable.
- Published/generated metadata preserves fixed 21x21 grid size.

---

## Phase 5 Prompt: Refactor Launch Orchestration
Paste this in chat:
Implement Phase 5 only. Refactor launch flow to deterministic order: Gazebo core, procedural maze spawn, robot spawn, SLAM, Nav2, RViz. Remove static map coupling in the default procedural path. Keep procedural grid size fixed at 21x21 in default mode.

Commands after implementation:
cd /home/prateek/Maze
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py --show-args
ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py

Expected outcome:
- New maze generated each launch.
- Navigation stack starts without relying on fixed map artifact for procedural mode.
- Each launch keeps fixed grid dimensions (21x21), while maze topology still randomizes.

---

## Phase 6 Prompt: Make Solver Dynamic
Paste this in chat:
Implement Phase 6 only. Replace hardcoded initial pose and goal in maze_solver with dynamic values from procedural pipeline. Add readiness wait/retry for Nav2 and endpoint availability. Ensure Nav2 global planner is configured to use A* (use_astar: true) and keep a practical, stable robot speed profile (no aggressive tuning).

Commands after implementation:
cd /home/prateek/Maze
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomous_tb3 maze_solver.py
ros2 topic echo /map --once
rg -n "use_astar|max_speed_xy|max_vel_x|controller_frequency" src/autonomous_tb3/config/tb3_nav_params.yaml

Expected outcome:
- Robot navigates from dynamic start to dynamic goal.
- Planner configuration confirms A* usage and speed remains okayish/stable for simulation.

---

## Phase 7 Prompt: End-to-End Repeated Validation
Paste this in chat:
Run Phase 7 validation only. Execute 5 runs, record success/failure and time-to-goal, and provide a summary table. Include observed peak linear speed to confirm motion is reasonable and stable.

Commands:
ros2 topic hz /map
ros2 node list
ros2 topic echo /tf --once
ros2 topic echo /cmd_vel --once
pkill -f gzserver; pkill -f gzclient; pkill -f rviz2

Expected outcome:
- Robustness evidence across multiple randomized maze runs.
- Summary includes success rate, time-to-goal, and speed sanity check.

---

## Phase 8 Prompt: Cleanup and Packaging
Paste this in chat:
Implement Phase 8 cleanup only. Update package metadata placeholders and verify tests/build quality. Keep runtime behavior unchanged.

Commands:
cd /home/prateek/Maze
rg -n "TODO: Package description|TODO: License declaration" src/autonomous_tb3/package.xml
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

Expected outcome:
- Project is cleaner for sharing and reproducible use.

---

## Safety Prompt You Can Reuse Any Time
Paste this in chat:
Before making any code changes, show me exactly which files you will modify in this phase, then proceed only for those files.

## Reporting Prompt You Can Reuse Any Time
Paste this in chat:
After this phase, give me: changed files list, exact command outputs summary, and next phase readiness status.
