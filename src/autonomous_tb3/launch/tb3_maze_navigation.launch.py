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


# Phase 5 – Refactored launch with deterministic startup order:
#   1. maze_generator.py  (writes /tmp/autonomous_tb3/maze_runtime.json)
#   2. Gazebo core (gzserver + gzclient)
#   3. robot_state_publisher
#   4. procedural_maze_spawner  (SDF → /spawn_entity; publishes start/goal topics)
#   5. spawn_turtlebot3         (places robot at procedural start cell)
#   6. Nav2 bringup with slam=true  (SLAM builds map at runtime; no static map file)
#   7. RViz2
#
# The static entity_spawner / maze_map.yaml coupling has been removed from this path.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_autonomous_tb3 = get_package_share_directory('autonomous_tb3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    launch_file_dir = os.path.join(pkg_tb3_gazebo, 'launch')
    params_config_file_path = os.path.join(pkg_autonomous_tb3, 'config', 'tb3_nav_params.yaml')
    rviz_config_file_path = os.path.join(pkg_autonomous_tb3, 'config', 'tb3_nav.rviz')

    # ── Declared launch arguments (visible via --show-args) ──────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    # Robot spawn position is fixed at the procedural maze start cell (1,1):
    #   world x = origin_x + (col+0.5)*resolution = -5.0 + 1.5*0.5 = -4.25
    #   world y = origin_y + (row+0.5)*resolution = -5.0 + 1.5*0.5 = -4.25
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='-4.25',
        description='Initial x-coordinate for TurtleBot3 (procedural maze start cell)',
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='-4.25',
        description='Initial y-coordinate for TurtleBot3 (procedural maze start cell)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # ── Environment ──────────────────────────────────────────────────────────
    setting_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle',
    )

    # ── Step 1: Generate the procedural maze artifact ─────────────────────────
    # maze_generator.py exits immediately after writing the JSON file, so
    # OnProcessExit fires as soon as the artifact is ready.
    maze_gen = ExecuteProcess(
        cmd=['ros2', 'run', 'autonomous_tb3', 'maze_generator.py'],
        output='screen',
        name='maze_generator',
    )

    # ── Steps 2-7: launched only after the maze artifact exists ──────────────

    # 2a. Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    # 2b. Gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    # 3. Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4. Procedural maze spawner:
    #    reads maze_runtime.json → builds SDF → calls /spawn_entity
    #    publishes /procedural_maze/start_pose and /procedural_maze/goal_pose
    procedural_maze_spawner = Node(
        package='autonomous_tb3',
        executable='procedural_maze_spawner.py',
        name='procedural_maze_spawner',
        output='screen',
    )

    # 5. Spawn TurtleBot3 at the procedural maze start cell
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    # 6. Nav2 bringup with SLAM enabled.
    #    slam=true makes nav2_bringup launch slam_toolbox (online_async) instead
    #    of map_server + AMCL, so no static map file is required.
    #    bringup_launch.py declares 'map' as a required argument (no default),
    #    so an empty string is passed to satisfy the declaration without loading
    #    a map (the value is unused when slam=true).
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'true',
            'map': '',
            'params_file': params_config_file_path,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # 7. RViz2
    rviz_launching = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_file_path],
        output='screen',
    )

    # Chain steps 2-7 behind the maze generator so the artifact is always
    # present before any downstream node tries to read it.
    post_gen_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=maze_gen,
            on_exit=[
                gzserver_cmd,
                gzclient_cmd,
                robot_state_publisher_cmd,
                procedural_maze_spawner,
                spawn_turtlebot_cmd,
                navigation,
                rviz_launching,
            ],
        )
    )

    ld = LaunchDescription()

    # Declare arguments first so --show-args works correctly.
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)

    ld.add_action(setting_turtlebot3_model)
    ld.add_action(maze_gen)
    ld.add_action(post_gen_handler)

    return ld