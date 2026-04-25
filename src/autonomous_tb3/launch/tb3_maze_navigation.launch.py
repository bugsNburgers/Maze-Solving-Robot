#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool


# Base code has been taken from the official Github account of Robotis: ROBOTIS-GIT/turtlebot3_simulations ('humble-devel' Branch)/turtlebot3_gazebo/launch/turtlebot3_world.launch.py  --- Modified by the myself (Pritam Rankan Kalita a.k.a preetamk97) as per my project requirements.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    params_config_file_path = os.path.join(get_package_share_directory('autonomous_tb3'), 'config', 'tb3_nav_params.yaml')
    rviz_config_file_path = os.path.join(get_package_share_directory('autonomous_tb3'), 'config', 'tb3_nav.rviz')
    runtime_map_path = '/tmp/autonomous_tb3/maze_runtime_map.yaml'
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='false')
    x_pose = LaunchConfiguration('x_pose', default='-4.25')    # Default start cell world-x for fixed 21x21 procedural maze.
    y_pose = LaunchConfiguration('y_pose', default='-4.25')    # Default start cell world-y for fixed 21x21 procedural maze.

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Enable SLAM mode (true) or runtime static-map mode (false).'
    )
    
    # To get the correct x & y coordinates for spawning the turtlebot3 robot, 
    
        ## firstly, launch this file while keeping the following line of code commented - "ld.add_action(spawn_turtlebot_cmd)"  [line can be found at the bottom part of this code].
        
        ## to launch this file
            ### open a terminal inside the workspace directory of this project.
            ### run the folllowing commands from the terminal:
                #### `source install/setup.bash`
                #### `ros2 launch autonomous_tb3 tb3_maze_navigation.launch.py`
        
        ## after doing the step mentioned above, this will render the maze world of this project -- inside the gazebo classic simulation environment -- but without the turtlebot3 robot in it.
        
        ## Now, inside the maze world simulation, place a unit box at the entry point of the maze - this will be the starting position of our robot when we start the simulation.
        
        ## Find the x & y coordinates of the unit box at == left-side-vertical panel of the Gazebo window > Models > unit_box > property-tab > pose.
        
        ## Copy the x & y coordinate values of the unit box from there, and paste them in the appropriate places in the code line 38 and 39 above.
        
    ## Uncomment the line of code which was previously commented - "ld.add_action(spawn_turtlebot_cmd)". Re-build the workspace. And launch this file once again. This time, you will see the complete simulation -- maze + turtlebot3 robot (which spawned in the same position where you placed the unit box previously).
    
    
    # Setting the type of turtlebot3 robot to be used in simulation.
    setting_turtlebot3_model = SetEnvironmentVariable(
        name = "TURTLEBOT3_MODEL",
        value="waffle"
    )

    # Avoid FastDDS shared-memory lock failures that can break topic exchange.
    disable_fastdds_shm = SetEnvironmentVariable(
        name="FASTDDS_BUILTIN_TRANSPORTS",
        value="UDPv4"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,   # x-coordinate for spawning the turtlebot3 robot inside the gazebo classic simulation environment.
            'y_pose': y_pose    # y-coordinate for spawning the turtlebot3 robot inside the gazebo classic simulation environment.
        }.items()
    )
    
    # Spawning maze world
    maze_spawner = Node(
        package = 'autonomous_tb3',
        executable = 'procedural_world_spawner.py',
        name = "maze_spawner",
        arguments = []
            # maze_path = path of the .sdf file for rendering the model
            # 'tb3_maze_world' = name of the model which can found inside the 'model.config' file for this model.
            # '0.0', '0.0' = x & y coordinates for spawning the model inside the gazebo classic simulation. 
    )
    
    # Launching Rviz2
    rviz_launching = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",    # node name can be anything of your choice
        arguments = ['-d', rviz_config_file_path],  # Only required for navigation purpose.
        output="screen"
    )
    
    # Using 'slam_toolbox' package's 'online_async_launch.py' launch file to for mapping the maze.
    maze_mapping_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (
            launch_file_path=os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')                        
        )        
    )
    
    # We can also use the 'mapping.launch.py' launch file of this package for using the cartographer_ros package for mapping purposes instead of slam_toolbox package. However, slam_toolbox is the recomended package for mapping in ros2.
    maze_mapping_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (
            launch_file_path=os.path.join(get_package_share_directory('autonomous_tb3'), 'launch', 'mapping.launch.py')                        
        )        
    )
    
    # Integrating Nav2 Stack for runtime static map mode.
    navigation_runtime_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (
            launch_file_path=os.path.join(get_package_share_directory('nav2_bringup'), "launch", "bringup_launch.py")
            
            # Using the 'bringup_launch.py' file inside the 'launch' directory of the 'nav2_bringup' package's 'share' directory - for using the Navigation2 stack with the project.
        ),
        launch_arguments = {
            'slam': 'False',
            'use_sim_time': 'True',
            'map': runtime_map_path,
            'params_file' : params_config_file_path
            }.items(),
        condition=UnlessCondition(use_slam),
    )

    # In SLAM mode, launch Nav2 with slam:=True so planner/controller are available.
    navigation_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (
            launch_file_path=os.path.join(get_package_share_directory('nav2_bringup'), "launch", "bringup_launch.py")
        ),
        launch_arguments = {
            'slam': 'True',
            'use_sim_time': 'True',
            'map': runtime_map_path,
            'params_file' : params_config_file_path
            }.items(),
        condition=IfCondition(use_slam),
    )
     

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(use_slam_arg)
    ld.add_action(setting_turtlebot3_model)
    ld.add_action(disable_fastdds_shm)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(maze_spawner)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(TimerAction(period=2.0, actions=[spawn_turtlebot_cmd]))
    # Nav2 bringup with slam:=True launches SLAM toolbox internally.
    ld.add_action(TimerAction(period=6.0, actions=[navigation_runtime_map]))
    ld.add_action(TimerAction(period=6.0, actions=[navigation_slam]))
    ld.add_action(TimerAction(period=8.0, actions=[rviz_launching]))
    # ld.add_action(maze_mapping_cartographer)
    
    
    return ld