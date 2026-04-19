#! /usr/bin/env python3

"""Spawn a fresh procedural maze model in Gazebo and publish dynamic endpoints."""

import json
import os
import random
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from script.maze_generator import MAZE_HEIGHT, MAZE_WIDTH, generate_maze


RUNTIME_PATH = "/tmp/autonomous_tb3/maze_runtime.json"
MAP_PGM_PATH = "/tmp/autonomous_tb3/maze_runtime_map.pgm"
MAP_YAML_PATH = "/tmp/autonomous_tb3/maze_runtime_map.yaml"


def _bfs_distances(maze, start):
    height = len(maze)
    width = len(maze[0])
    queue = deque([start])
    distances = {start: 0}

    while queue:
        row, col = queue.popleft()
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr = row + dr
            nc = col + dc
            if 0 <= nr < height and 0 <= nc < width and maze[nr][nc] == 0:
                nxt = (nr, nc)
                if nxt not in distances:
                    distances[nxt] = distances[(row, col)] + 1
                    queue.append(nxt)

    return distances


def _safe_interior_goal_cell(distances, fallback, margin=2, height=None, width=None):
    if height is None or width is None:
        raise ValueError("height and width are required")

    interior = {
        cell: dist
        for cell, dist in distances.items()
        if margin <= cell[0] < height - margin and margin <= cell[1] < width - margin
    }
    if interior:
        return max(interior.items(), key=lambda item: item[1])[0]

    if distances:
        return max(distances.items(), key=lambda item: item[1])[0]

    return fallback


def _reachable_short_hop_goal(distances, fallback, min_steps=6, max_steps=14):
    candidates = [
        cell for cell, dist in distances.items() if min_steps <= dist <= max_steps
    ]
    if candidates:
        # Prefer the farthest candidate in this short-hop band.
        return max(candidates, key=lambda cell: distances[cell])
    return fallback


def _to_world_cell(cell, resolution=0.5, origin=(-5.0, -5.0)):
    row, col = cell
    ox, oy = origin
    x = ox + (col + 0.5) * resolution
    y = oy + (row + 0.5) * resolution
    return {"x": round(x, 3), "y": round(y, 3), "yaw": 0.0}


def _create_runtime_payload():
    seed = random.randrange(1, 10_000_000)
    maze = generate_maze(height=MAZE_HEIGHT, width=MAZE_WIDTH, seed=seed)
    start_cell = (1, 1)
    distances = _bfs_distances(maze, start_cell)
    farthest_cell = max(distances.items(), key=lambda item: item[1])[0]
    interior_goal = _safe_interior_goal_cell(
        distances,
        fallback=farthest_cell,
        margin=2,
        height=len(maze),
        width=len(maze[0]),
    )
    # Keep finish line far, but cap route length to remain practical for runtime tests.
    goal_cell = _reachable_short_hop_goal(
        distances,
        fallback=interior_goal,
        min_steps=35,
        max_steps=60,
    )

    return {
        "seed": seed,
        "height": len(maze),
        "width": len(maze[0]),
        "resolution": 0.5,
        "origin": {"x": -5.0, "y": -5.0},
        "start_cell": {"row": start_cell[0], "col": start_cell[1]},
        "goal_cell": {"row": goal_cell[0], "col": goal_cell[1]},
        "start_pose": _to_world_cell(start_cell),
        "goal_pose": _to_world_cell(goal_cell),
        "grid": maze,
    }


def _save_runtime_payload(payload, output_path=RUNTIME_PATH):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)


def _save_runtime_map(payload, pgm_path=MAP_PGM_PATH, yaml_path=MAP_YAML_PATH):
    os.makedirs(os.path.dirname(pgm_path), exist_ok=True)

    grid = payload["grid"]
    height = len(grid)
    width = len(grid[0])

    # PGM map values: 0=occupied, 254=free.
    with open(pgm_path, "w", encoding="ascii") as pgm:
        pgm.write("P2\n")
        pgm.write(f"{width} {height}\n")
        pgm.write("255\n")
        # Align map raster orientation with Gazebo world coordinates.
        for row in reversed(grid):
            values = ["0" if cell == 1 else "254" for cell in row]
            pgm.write(" ".join(values) + "\n")

    origin_x = float(payload["origin"]["x"])
    origin_y = float(payload["origin"]["y"])
    resolution = float(payload["resolution"])

    yaml_content = "\n".join([
        f"image: {pgm_path}",
        "mode: trinary",
        f"resolution: {resolution}",
        f"origin: [{origin_x}, {origin_y}, 0.0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
        "",
    ])
    with open(yaml_path, "w", encoding="ascii") as yaml_file:
        yaml_file.write(yaml_content)


def _build_maze_sdf(payload):
    grid = payload["grid"]
    resolution = float(payload["resolution"])
    origin_x = float(payload["origin"]["x"])
    origin_y = float(payload["origin"]["y"])
    wall_height = 1.0
    wall_size = f"{resolution:.3f} {resolution:.3f} {wall_height:.3f}"

    wall_blocks = []
    wall_count = 0
    for row, row_values in enumerate(grid):
        for col, value in enumerate(row_values):
            if value != 1:
                continue
            wall_count += 1
            x = origin_x + (col + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution
            z = wall_height / 2.0

            wall_blocks.append(
                f"""
      <collision name='wall_collision_{wall_count}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <geometry>
          <box><size>{wall_size}</size></box>
        </geometry>
      </collision>
      <visual name='wall_visual_{wall_count}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <geometry>
          <box><size>{wall_size}</size></box>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.30 0.30 0.30 1</diffuse>
        </material>
      </visual>
                """.rstrip()
            )

    start = payload["start_pose"]
    goal = payload["goal_pose"]
    marker_radius = max(0.08, resolution * 0.18)

    markers = f"""
      <visual name='start_marker'>
        <pose>{start['x']:.3f} {start['y']:.3f} 0.05 0 0 0</pose>
        <geometry>
          <sphere><radius>{marker_radius:.3f}</radius></sphere>
        </geometry>
        <material>
          <ambient>0.0 0.8 0.0 1</ambient>
          <diffuse>0.0 0.8 0.0 1</diffuse>
        </material>
      </visual>
      <visual name='goal_marker'>
        <pose>{goal['x']:.3f} {goal['y']:.3f} 0.05 0 0 0</pose>
        <geometry>
          <sphere><radius>{marker_radius:.3f}</radius></sphere>
        </geometry>
        <material>
          <ambient>0.9 0.2 0.1 1</ambient>
          <diffuse>0.9 0.2 0.1 1</diffuse>
        </material>
      </visual>
    """.rstrip()

    maze_xml = "\n".join(wall_blocks + [markers])
    return f"""
<sdf version='1.7'>
  <model name='procedural_maze'>
    <static>true</static>
    <link name='maze_link'>
{maze_xml}
    </link>
  </model>
</sdf>
""".strip()


class ProceduralWorldSpawner(Node):
    def __init__(self):
        super().__init__("procedural_world_spawner")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_pub = self.create_publisher(PoseStamped, "/procedural_maze/start", qos)
        self.goal_pub = self.create_publisher(PoseStamped, "/procedural_maze/goal", qos)

        self.payload = _create_runtime_payload()
        _save_runtime_payload(self.payload)
        _save_runtime_map(self.payload)
        self.get_logger().info(
            f"Generated runtime maze seed={self.payload['seed']} size={self.payload['height']}x{self.payload['width']}"
        )

        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self._spawn_maze_model()

        self.publish_endpoints()
        self.create_timer(1.0, self.publish_endpoints)

    def _spawn_maze_model(self):
        self.get_logger().info("Waiting for /spawn_entity service...")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/spawn_entity service not available yet...")

        request = SpawnEntity.Request()
        request.name = f"procedural_maze_{self.payload['seed']}"
        request.xml = _build_maze_sdf(self.payload)
        request.robot_namespace = ""
        request.reference_frame = "world"

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError(f"Failed to spawn procedural maze: {future.exception()}")

        response = future.result()
        if not response.success:
            raise RuntimeError(f"Spawn request failed: {response.status_message}")

        self.get_logger().info(f"Spawned model '{request.name}'")

    def publish_endpoints(self):
        stamp = self.get_clock().now().to_msg()

        start_msg = PoseStamped()
        start_msg.header.frame_id = "map"
        start_msg.header.stamp = stamp
        start_msg.pose.position.x = float(self.payload["start_pose"]["x"])
        start_msg.pose.position.y = float(self.payload["start_pose"]["y"])
        start_msg.pose.orientation.w = 1.0

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = stamp
        goal_msg.pose.position.x = float(self.payload["goal_pose"]["x"])
        goal_msg.pose.position.y = float(self.payload["goal_pose"]["y"])
        goal_msg.pose.orientation.w = 1.0

        self.start_pub.publish(start_msg)
        self.goal_pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProceduralWorldSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()