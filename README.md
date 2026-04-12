# Autonomous Maze Solving Turtlebot3 Simulation

This repository contains code for a **Gazebo Classic** simulation of a **Turtlebot3** (***waffle***) robot (designed by *Robotis*) that navigates a maze **autonomously**.

This project was developed by following concepts from the **Udemy** course **ROS2 Autonomous Driving and SLAM using NAV2 with TurtleBot3** by **Muhammad Luqman**.

Main software used in this project:

- Python3 Interpreter
- Basic ROS2 Framework
- Gazebo Classic Simulator
- *turtlebot3_gazebo* ROS2 package
- *slam_toolbox* ROS2 package
- Navigation2 Stack

Operating system:
- Ubuntu (tested on **Ubuntu 22.04** with **ROS 2 Humble**)

To run this project successfully, install all of the above on your Ubuntu machine.


## What is different in this project?

In the original course flow, this project is created as an **ament_python** ROS2 package. In this repository, it is implemented as an **ament_cmake** ROS2 package.


## Deployment

Follow these steps to deploy and run the project.

- **Create a new folder** anywhere in your Ubuntu system. In this guide, it is named **Cloned Repo**.

- Open a **new terminal** inside that folder.

- **Clone this repository** by running:

    ```bash
    git clone https://github.com/bugsNburgers/Maze-Solving-Robot.git
    ```

- This creates a folder named **Maze-Solving-Robot** inside **Cloned Repo**.

- Go into the **Maze-Solving-Robot** folder.

    ```
    cd Maze-Solving-Robot/
    ```

- Install dependencies (Gazebo Classic, Turtlebot3 simulation, Nav2, and the required `colcon` ROS plugin).

    ```bash
    sudo apt-get update
    grep -v '^#' reqs.txt | xargs sudo apt-get install -y
    ```

    Notes:
    - This assumes you already have ROS 2 Humble installed and your ROS apt repo set up.
    - If you see `colcon build` finishing with **0 packages**, you are missing `python3-colcon-ros` (included in `reqs.txt`).

- Ensure the Python scripts are executable (required because this is an `ament_cmake` package and scripts are installed via CMake):

    ```bash
    chmod +x src/autonomous_tb3/script/*.py
    ```

- Build the project using the same terminal:

    ```bash
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    ```
    
    This generates three folders in the project directory: **build**, **install**, and **log**.

- Close that terminal.

- Start the simulation:
    
    - Open a new terminal (Terminal 1) in the **Maze-Solving-Robot** directory and run:
        ```bash
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py
        ```

        This opens a **Gazebo Classic** window (with the ***maze world*** and a ***Turtlebot3*** robot) and an **RViz2** window (with the 2D map).
        
        Use the ***mouse scroll wheel*** to ***zoom in/out*** in both **Gazebo** and **RViz2**.

        In **Gazebo**, use the **left mouse button** to pan and the ***scroll button*** to rotate the view.

        In **RViz2**, use the **left mouse button** to rotate and the ***scroll button*** to pan the map view.

        Before continuing, it is recommended to keep **Gazebo** and **RViz2** side by side so you can observe both views together.

    - Wait until Gazebo and RViz are fully open, then open a second terminal (Terminal 2) in the same directory (keep Terminal 1 running) and run:
        ```bash
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 run autonomous_tb3 maze_solver.py
        ```

    - If everything is set up correctly, the robot will start moving in Gazebo, and the solver will print messages like:
        - `Nav2 is ready for use!`
        - `Navigating to goal ...`

    Quick troubleshooting:
    - If you get "Entity [waffle] already exists", kill stale sim processes and relaunch:
      `pkill -f gzserver; pkill -f gzclient; pkill -f rviz2`
    - If the solver says `amcl/get_state service not available`, the Nav2 stack is not fully active (or Terminal 1 was stopped). Keep the launch running.


## Output

After following the deployment steps, you should see simulation output like this in **Gazebo Classic**.

<figure class="video_container">
  <video controls="true" allowfullscreen="true" poster="Thumbnail.png">
    <source src="Gazebo_Sim_Recording.mp4" type="video/mp4">
  </video>
</figure>

At the same time, the robot follows the same trajectory in the **2D maze map** in **RViz2**. A separate RViz recording is not included due to hardware limitations.
