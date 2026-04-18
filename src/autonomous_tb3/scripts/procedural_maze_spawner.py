#! /usr/bin/env python3

"""Phase 4 – Procedural world spawner.

Reads /tmp/autonomous_tb3/maze_runtime.json (written by maze_generator.py),
converts every wall cell into a Gazebo SDF box model, spawns the whole maze
via the /spawn_entity service, and then continuously publishes the dynamic
start and goal poses so that the maze solver (and any other node) can consume
them without hard-coding any coordinates.

Published topics (transient-local, so late-joining nodes still receive them):
  /procedural_maze/start_pose  – geometry_msgs/msg/PoseStamped
  /procedural_maze/goal_pose   – geometry_msgs/msg/PoseStamped
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ARTIFACT_PATH = "/tmp/autonomous_tb3/maze_runtime.json"
WALL_HEIGHT = 0.5          # meters - taller than TurtleBot3 lidar at ~0.18 m
MODEL_NAME = "procedural_maze"
PUBLISH_RATE_HZ = 1.0      # how often to re-publish start/goal (seconds)


# ---------------------------------------------------------------------------
# SDF builder
# ---------------------------------------------------------------------------

def _build_sdf(grid, resolution, origin_x, origin_y):
    """Return an SDF string representing every wall cell as a static box."""
    height = len(grid)
    width = len(grid[0])
    half_h = WALL_HEIGHT / 2.0

    links = []
    for row in range(height):
        for col in range(width):
            if grid[row][col] != 1:
                continue
            cx = origin_x + (col + 0.5) * resolution
            cy = origin_y + (row + 0.5) * resolution
            link_name = f"wall_r{row:02d}_c{col:02d}"
            links.append(
                f"""    <link name='{link_name}'>
      <pose>{cx:.4f} {cy:.4f} {half_h:.4f} 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box><size>{resolution} {resolution} {WALL_HEIGHT}</size></box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box><size>{resolution} {resolution} {WALL_HEIGHT}</size></box>
        </geometry>
        <material>
          <ambient>0.45 0.45 0.45 1</ambient>
          <diffuse>0.45 0.45 0.45 1</diffuse>
        </material>
      </visual>
    </link>"""
            )

    links_xml = "\n".join(links)
    return f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='{MODEL_NAME}'>
    <static>true</static>
{links_xml}
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class ProceduralMazeSpawner(Node):
    """Spawns the procedural maze and publishes start/goal poses."""

    def __init__(self):
        super().__init__("procedural_maze_spawner")

        # Transient-local QoS: late-joining subscribers receive the last msg.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._start_pub = self.create_publisher(
            PoseStamped, "/procedural_maze/start_pose", latched_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, "/procedural_maze/goal_pose", latched_qos
        )

        self._artifact = self._load_artifact()

        self._spawn_maze()

        # Publish initial poses immediately.
        self._publish_poses()

        # Keep re-publishing so nodes that join later don't miss them.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_poses)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _load_artifact(self):
        if not os.path.exists(ARTIFACT_PATH):
            self.get_logger().fatal(
                f"Maze artifact not found at {ARTIFACT_PATH}. "
                "Run maze_generator.py first."
            )
            raise FileNotFoundError(ARTIFACT_PATH)

        with open(ARTIFACT_PATH, "r", encoding="utf-8") as fh:
            data = json.load(fh)

        self.get_logger().info(
            f"Loaded maze artifact – seed={data['seed']}  "
            f"size={data['height']}x{data['width']}"
        )
        return data

    def _spawn_maze(self):
        data = self._artifact
        sdf = _build_sdf(
            data["grid"],
            data["resolution"],
            data["origin"]["x"],
            data["origin"]["y"],
        )

        client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for /spawn_entity service ...")
        client.wait_for_service()
        self.get_logger().info("/spawn_entity ready - spawning procedural maze")

        request = SpawnEntity.Request()
        request.name = MODEL_NAME
        request.xml = sdf
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.0

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f"Spawn result: {future.result().status_message}"
            )
        else:
            self.get_logger().error(
                f"Spawn failed: {future.exception()}"
            )

    def _make_pose_stamped(self, pose_dict):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = pose_dict["x"]
        msg.pose.position.y = pose_dict["y"]
        msg.pose.position.z = 0.0
        yaw = pose_dict.get("yaw", 0.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def _publish_poses(self):
        self._start_pub.publish(
            self._make_pose_stamped(self._artifact["start_pose"])
        )
        self._goal_pub.publish(
            self._make_pose_stamped(self._artifact["goal_pose"])
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ProceduralMazeSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
