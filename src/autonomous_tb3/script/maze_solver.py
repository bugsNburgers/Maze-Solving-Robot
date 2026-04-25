#! /usr/bin/env python3

"""Navigate from dynamic procedural start to dynamic procedural goal."""

import json
import os
import time
from collections import deque
from math import cos, sin
from math import atan2, pi

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


RUNTIME_PATH = "/tmp/autonomous_tb3/maze_runtime.json"


def compute_shortest_path(grid, start_cell, goal_cell):
    """Return shortest 4-connected grid path from start to goal as a list of cells."""
    height = len(grid)
    width = len(grid[0])
    queue = deque([start_cell])
    parent = {start_cell: None}

    while queue:
        row, col = queue.popleft()
        if (row, col) == goal_cell:
            break
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr = row + dr
            nc = col + dc
            nxt = (nr, nc)
            if 0 <= nr < height and 0 <= nc < width and grid[nr][nc] == 0 and nxt not in parent:
                parent[nxt] = (row, col)
                queue.append(nxt)

    if goal_cell not in parent:
        return []

    path = []
    cursor = goal_cell
    while cursor is not None:
        path.append(cursor)
        cursor = parent[cursor]
    path.reverse()
    return path


def grid_cell_to_pose(cell, payload):
    """Convert a maze cell (row, col) into map-frame world pose dictionary."""
    row, col = cell
    resolution = float(payload["resolution"])
    ox = float(payload["origin"]["x"])
    oy = float(payload["origin"]["y"])
    return {
        "x": ox + (col + 0.5) * resolution,
        "y": oy + (row + 0.5) * resolution,
        "yaw": 0.0,
    }


def build_segment_goals(shortest_path, payload, stride=2):
    """Subsample path into intermediate goals to reduce local planner burden."""
    if not shortest_path:
        return []

    goals = [shortest_path[0]]
    for idx in range(stride, len(shortest_path), stride):
        goals.append(shortest_path[idx])
    if goals[-1] != shortest_path[-1]:
        goals.append(shortest_path[-1])
    return [grid_cell_to_pose(cell, payload) for cell in goals[1:]]


class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solver")
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._feedback_counter = 0
        self._odom = None
        self._slam_mode = False
        self._slam_alignment = None

    def _odom_callback(self, msg):
        self._odom = msg

    def wait_for_odom(self, timeout_sec=20.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self._odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom is None:
            raise RuntimeError("Timed out waiting for /odom")

    def wait_for_cmd_vel_subscriber(self, timeout_sec=20.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.count_subscribers("/cmd_vel") > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        raise RuntimeError("Timed out waiting for /cmd_vel subscriber (turtlebot3_diff_drive)")

    def wait_for_map_tf(self, timeout_sec=20.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._xyyaw_from_map_tf() is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        raise RuntimeError("Timed out waiting for map->base_link TF")

    def _yaw_from_odom(self):
        if self._odom is None:
            return 0.0
        q = self._odom.pose.pose.orientation
        return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _xy_from_odom(self):
        if self._odom is None:
            return 0.0, 0.0
        p = self._odom.pose.pose.position
        return p.x, p.y

    def _xyyaw_from_map_tf(self):
        """Return robot (x, y, yaw) in map frame from TF, or None if unavailable."""
        try:
            tf_msg = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    @staticmethod
    def _angle_diff(target, current):
        diff = target - current
        while diff > pi:
            diff -= 2.0 * pi
        while diff < -pi:
            diff += 2.0 * pi
        return diff

    def _build_slam_alignment(self, payload):
        """Build a rigid transform from runtime map coordinates to live SLAM map frame."""
        map_pose = self._xyyaw_from_map_tf()
        if map_pose is None:
            self.get_logger().warn("Cannot build SLAM alignment yet: map->base_link TF unavailable")
            return False

        observed_x, observed_y, observed_yaw = map_pose
        start_pose = payload["start_pose"]
        expected_x = float(start_pose["x"])
        expected_y = float(start_pose["y"])
        expected_yaw = float(start_pose.get("yaw", 0.0))

        delta_yaw = self._angle_diff(observed_yaw, expected_yaw)
        self._slam_alignment = {
            "expected_x": expected_x,
            "expected_y": expected_y,
            "observed_x": observed_x,
            "observed_y": observed_y,
            "delta_yaw": delta_yaw,
        }
        self.get_logger().info(
            f"SLAM alignment locked (dx={observed_x - expected_x:.2f}, dy={observed_y - expected_y:.2f}, dyaw={delta_yaw:.2f} rad)"
        )
        return True

    def _align_pose_for_slam(self, pose_dict):
        if not self._slam_mode or self._slam_alignment is None:
            return dict(pose_dict)

        px = float(pose_dict["x"])
        py = float(pose_dict["y"])
        pyaw = float(pose_dict.get("yaw", 0.0))

        exp_x = self._slam_alignment["expected_x"]
        exp_y = self._slam_alignment["expected_y"]
        obs_x = self._slam_alignment["observed_x"]
        obs_y = self._slam_alignment["observed_y"]
        dyaw = self._slam_alignment["delta_yaw"]

        rx = px - exp_x
        ry = py - exp_y
        ax = obs_x + (cos(dyaw) * rx - sin(dyaw) * ry)
        ay = obs_y + (sin(dyaw) * rx + cos(dyaw) * ry)
        ayaw = pyaw + dyaw

        return {"x": ax, "y": ay, "yaw": ayaw}

    def _stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def _unstick_maneuver(self):
        """Short reverse-and-turn maneuver to escape local deadlocks near corners."""
        self.get_logger().warn("Applying unstick maneuver")

        end_t = time.time() + 0.7
        while time.time() < end_t:
            msg = Twist()
            msg.linear.x = -0.05
            msg.angular.z = 0.45
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self._stop_robot()

    def rotate_to_heading(self, target_yaw, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            current = self._yaw_from_odom()
            err = self._angle_diff(target_yaw, current)
            if abs(err) < 0.06:
                self._stop_robot()
                return True

            msg = Twist()
            msg.angular.z = max(min(1.4 * err, 0.8), -0.8)
            self.cmd_vel_pub.publish(msg)

        self._stop_robot()
        return False

    def drive_forward_distance(self, distance, timeout_sec=12.0):
        if self._odom is None:
            return False
        start_x = self._odom.pose.pose.position.x
        start_y = self._odom.pose.pose.position.y
        deadline = time.time() + timeout_sec

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is None:
                continue
            dx = self._odom.pose.pose.position.x - start_x
            dy = self._odom.pose.pose.position.y - start_y
            traveled = (dx * dx + dy * dy) ** 0.5
            if traveled >= distance - 0.02:
                self._stop_robot()
                return True

            msg = Twist()
            msg.linear.x = 0.10
            self.cmd_vel_pub.publish(msg)

        self._stop_robot()
        return False

    def drive_to_waypoint(self, target_x, target_y, timeout_sec=18.0):
        """Closed-loop go-to-waypoint controller using map TF when available."""
        deadline = time.time() + timeout_sec
        best_distance = float("inf")
        stagnation_start = time.time()
        tf_missing_since = None

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            map_pose = self._xyyaw_from_map_tf()
            if map_pose is not None:
                cx, cy, yaw = map_pose
                tf_missing_since = None
            else:
                # In SLAM mode the path is map-frame; do not drive with odom-frame fallback.
                if self._slam_mode:
                    if tf_missing_since is None:
                        tf_missing_since = time.time()
                    if time.time() - tf_missing_since > 2.0:
                        self._stop_robot()
                        return False
                    continue
                cx, cy = self._xy_from_odom()
                yaw = self._yaw_from_odom()
            dx = target_x - cx
            dy = target_y - cy
            distance = (dx * dx + dy * dy) ** 0.5
            if distance < 0.12:
                self._stop_robot()
                return True

            if distance + 0.005 < best_distance:
                best_distance = distance
                stagnation_start = time.time()
            elif time.time() - stagnation_start > 2.5:
                self._unstick_maneuver()
                stagnation_start = time.time()
                continue

            target_yaw = atan2(dy, dx)
            yaw_error = self._angle_diff(target_yaw, yaw)

            cmd = Twist()
            if abs(yaw_error) > 0.35:
                cmd.linear.x = 0.0
                cmd.angular.z = max(min(1.5 * yaw_error, 0.9), -0.9)
            else:
                cmd.linear.x = min(0.13, max(0.04, 0.35 * distance))
                cmd.angular.z = max(min(1.2 * yaw_error, 0.8), -0.8)
            self.cmd_vel_pub.publish(cmd)

        self._stop_robot()
        return False

    def wait_for_runtime_payload(self, timeout_sec=60.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if os.path.exists(RUNTIME_PATH):
                try:
                    with open(RUNTIME_PATH, "r", encoding="utf-8") as file:
                        payload = json.load(file)
                    if payload.get("start_pose") and payload.get("goal_pose"):
                        return payload
                except (json.JSONDecodeError, OSError):
                    pass
            time.sleep(0.5)
        raise RuntimeError(f"Timed out waiting for runtime payload at {RUNTIME_PATH}")

    def wait_for_endpoint_topics(self, timeout_sec=30.0):
        required_topics = ["/procedural_maze/start", "/procedural_maze/goal"]
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if all(self.count_publishers(topic) > 0 for topic in required_topics):
                return
            time.sleep(0.5)
        raise RuntimeError("Timed out waiting for procedural endpoint publishers")

    def wait_for_navigate_server(self, start_pose, timeout_sec=90.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            # Keep nudging AMCL so lifecycle-managed Nav2 nodes can transition to active.
            self.publish_initial_pose(start_pose)
            if self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
                return
            time.sleep(0.5)
        raise RuntimeError("Timed out waiting for /navigate_to_pose action server")

    def _pose_stamped(self, pose_dict):
        yaw = float(pose_dict.get("yaw", 0.0))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_dict["x"])
        pose.pose.position.y = float(pose_dict["y"])
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        return pose

    def publish_initial_pose(self, pose_dict):
        pose = self._pose_stamped(pose_dict)
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = pose.header
        initial_pose.pose.pose = pose.pose
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.06853892326654787

        for _ in range(8):
            # Use zero time to let AMCL transform at the latest available TF timestamp.
            initial_pose.header.stamp.sec = 0
            initial_pose.header.stamp.nanosec = 0
            self.initial_pose_pub.publish(initial_pose)
            rclpy.spin_once(self, timeout_sec=0.1)

    def feedback_callback(self, feedback_msg):
        self._feedback_counter += 1
        feedback = feedback_msg.feedback
        if feedback and self._feedback_counter % 25 == 0:
            eta = feedback.estimated_time_remaining.sec
            self.get_logger().info(f"Estimated time of arrival: {eta} seconds")

    def navigate(self, goal_pose, start_pose, max_attempts=1, segment_timeout_sec=18.0):
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        while not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("/navigate_to_pose not available yet...")

        for attempt in range(1, max_attempts + 1):
            # Republish initial pose on each attempt so AMCL can latch once it is active.
            self.publish_initial_pose(start_pose)
            time.sleep(0.5)
            self.get_logger().info(f"Sending goal attempt {attempt}/{max_attempts}")
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._pose_stamped(goal_pose)

            send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, self.feedback_callback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by Nav2")
                time.sleep(2.0)
                continue

            result_future = goal_handle.get_result_async()
            deadline = time.time() + segment_timeout_sec
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.5)
                if time.time() > deadline:
                    self.get_logger().error("Segment timed out before Nav2 returned a result")
                    goal_handle.cancel_goal_async()
                    break

            if not result_future.done() or result_future.result() is None:
                self.get_logger().error("Did not receive navigation result")
                time.sleep(2.0)
                continue

            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Goal succeeded!")
                return True
            if status == GoalStatus.STATUS_CANCELED:
                self.get_logger().error("Goal was canceled")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("Goal failed (aborted)")
            else:
                self.get_logger().error(f"Goal ended with unexpected status code: {status}")
            time.sleep(2.0)

        return False

    def _nearest_path_index(self, path_cells, payload):
        map_pose = self._xyyaw_from_map_tf()
        if map_pose is not None:
            cx, cy, _ = map_pose
        else:
            cx, cy = self._xy_from_odom()
        best_idx = 0
        best_dist = float("inf")
        for idx, cell in enumerate(path_cells):
            pose = self._align_pose_for_slam(grid_cell_to_pose(cell, payload))
            dx = pose["x"] - cx
            dy = pose["y"] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_dist:
                best_dist = d2
                best_idx = idx
        return best_idx

    def _is_slam_mode(self):
        """Infer SLAM mode from active graph nodes/topics."""
        try:
            for name, _ in self.get_node_names_and_namespaces():
                if name in ("slam_toolbox", "/slam_toolbox"):
                    return True
        except Exception:
            pass
        return self.count_publishers("/slam_toolbox/update") > 0

    def _direct_waypoint_indices(self, shortest_path, start_idx, stride=2):
        indices = []
        i = start_idx + 1
        while i < len(shortest_path):
            indices.append(i)
            i += stride
        if indices and indices[-1] != len(shortest_path) - 1:
            indices.append(len(shortest_path) - 1)
        elif not indices and start_idx < len(shortest_path) - 1:
            indices.append(len(shortest_path) - 1)
        return indices

    def navigate_path_direct(self, shortest_path, payload):
        self.wait_for_cmd_vel_subscriber()
        if self._slam_mode:
            self.wait_for_map_tf()
        else:
            self.wait_for_odom()
        start_idx = self._nearest_path_index(shortest_path, payload)
        failed_waypoint_count = 0
        self.get_logger().info(
            f"Starting direct waypoint fallback from path index {start_idx}/{len(shortest_path)-1}"
        )

        waypoint_indices = self._direct_waypoint_indices(shortest_path, start_idx, stride=2)
        cursor = 0

        while cursor < len(waypoint_indices):
            i = waypoint_indices[cursor]
            waypoint = self._align_pose_for_slam(grid_cell_to_pose(shortest_path[i], payload))
            ok = self.drive_to_waypoint(waypoint["x"], waypoint["y"], timeout_sec=18.0)
            if not ok:
                failed_waypoint_count += 1
                self.get_logger().warn(
                    f"Direct waypoint controller failed at cell index {i}; attempting path re-anchor"
                )

                # Unstick once more, then continue from nearest path point to current pose.
                self._unstick_maneuver()
                nearest_idx = self._nearest_path_index(shortest_path, payload)
                # Rebuild sparse forward waypoints from the new anchor so we do not grind on dense corners.
                waypoint_indices = self._direct_waypoint_indices(shortest_path, max(i, nearest_idx), stride=2)
                cursor = 0

                if failed_waypoint_count >= 8:
                    self.get_logger().error("Too many direct waypoint failures; aborting direct controller")
                    return False
                continue

            cursor += 1

        self._stop_robot()
        self.get_logger().info("Reached destination via direct fallback controller")
        return True

    def navigate_path(self, payload):
        start_cell = (int(payload["start_cell"]["row"]), int(payload["start_cell"]["col"]))
        goal_cell = (int(payload["goal_cell"]["row"]), int(payload["goal_cell"]["col"]))
        shortest_path = compute_shortest_path(payload["grid"], start_cell, goal_cell)
        if not shortest_path:
            raise RuntimeError("No grid path exists between start and goal in runtime payload")

        slam_mode = self._is_slam_mode()
        self._slam_mode = slam_mode
        if slam_mode:
            # Try a few times in case SLAM map->base_link TF is still warming up.
            for _ in range(20):
                if self._build_slam_alignment(payload):
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            self.get_logger().info(
                f"Computed shortest path with {len(shortest_path)-1} steps; SLAM detected"
            )

            # Prefer Nav2 in SLAM mode if available; it handles narrow corridors better.
            if self.navigate_to_pose_client.wait_for_server(timeout_sec=2.0):
                # First try a single global goal; let Nav2 compute the full corridor path.
                start_pose = payload["start_pose"]
                aligned_goal = self._align_pose_for_slam(payload["goal_pose"])
                self.get_logger().info(
                    f"SLAM/Nav2 global goal to ({aligned_goal['x']:.2f}, {aligned_goal['y']:.2f})"
                )
                ok = self.navigate(aligned_goal, start_pose, max_attempts=3, segment_timeout_sec=180.0)
                if ok:
                    return True

                # If a single global goal fails, fall back to segmented goals.
                self.get_logger().warn("SLAM/Nav2 global goal failed; trying segmented Nav2 goals")
                segment_goals = build_segment_goals(shortest_path, payload, stride=3)
                for index, goal_pose in enumerate(segment_goals, start=1):
                    aligned_segment = self._align_pose_for_slam(goal_pose)
                    self.get_logger().info(
                        f"SLAM/Nav2 segment {index}/{len(segment_goals)} to ({aligned_segment['x']:.2f}, {aligned_segment['y']:.2f})"
                    )
                    ok = self.navigate(aligned_segment, start_pose, max_attempts=2, segment_timeout_sec=90.0)
                    if not ok:
                        self.get_logger().warn("SLAM/Nav2 segment failed; falling back to direct waypoint controller")
                        break
                    start_pose = aligned_segment
                else:
                    return True

            self.get_logger().info("Using TF-aligned direct route fallback in SLAM mode")
        else:
            self.get_logger().info(
                f"Computed shortest path with {len(shortest_path)-1} steps; starting direct route execution"
            )

        if self.navigate_path_direct(shortest_path, payload):
            return True

        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Direct route failed and /navigate_to_pose is unavailable")
            return False

        self.get_logger().warn("Direct route execution failed; falling back to Nav2 segmented goals")

        segment_goals = build_segment_goals(shortest_path, payload, stride=2)
        self.get_logger().info(
            f"Computed shortest path with {len(shortest_path)-1} steps and {len(segment_goals)} segment goals"
        )

        start_pose = payload["start_pose"]
        for index, goal_pose in enumerate(segment_goals, start=1):
            self.get_logger().info(
                f"Navigating segment {index}/{len(segment_goals)} to ({goal_pose['x']:.2f}, {goal_pose['y']:.2f})"
            )
            ok = self.navigate(goal_pose, start_pose, max_attempts=2, segment_timeout_sec=60.0)
            if not ok:
                self.get_logger().warn("Nav2 segment execution failed; switching to direct waypoint fallback")
                return self.navigate_path_direct(shortest_path, payload)
            # After the first segment, use last reached pose as next recovery seed.
            start_pose = goal_pose
        return True


def main():
    rclpy.init()
    solver = MazeSolver()

    try:
        solver.wait_for_endpoint_topics()
        payload = solver.wait_for_runtime_payload()
        solver.publish_initial_pose(payload["start_pose"])
        success = solver.navigate_path(payload)
        if success:
            solver.get_logger().info("Reached destination across all path segments")
        else:
            solver.get_logger().error("Failed to complete one or more path segments")
    except Exception as exc:
        solver.get_logger().error(f"Maze solver stopped with error: {exc}")
    finally:
        solver.destroy_node()
        # Guard against duplicate shutdown when interrupted by external timeouts/signals.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
