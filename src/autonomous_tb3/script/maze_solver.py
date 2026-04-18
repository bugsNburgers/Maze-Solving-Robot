#! /usr/bin/env python3

# Phase 6 – Dynamic maze solver.
#
# Reads start/goal poses from the procedural maze pipeline
# (/procedural_maze/start_pose, /procedural_maze/goal_pose) instead of
# hardcoded coordinates, waits for Nav2 readiness, then drives the robot
# from start to goal.
#
# Base code copied from the Official Github Account of ROS Planning :  ros-planning/navigation2 ('humble' branch)/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py
# Modified as per the current project needs.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


TOPIC_START = '/procedural_maze/start_pose'
TOPIC_GOAL = '/procedural_maze/goal_pose'
POSE_WAIT_TIMEOUT_S = 60.0   # seconds to wait for each pose topic


def _wait_for_pose(node, topic, timeout_s):
    """Block until one message arrives on *topic* (transient-local) or raise."""
    latched_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    received = []

    def _cb(msg):
        received.append(msg)

    sub = node.create_subscription(PoseStamped, topic, _cb, latched_qos)
    deadline = node.get_clock().now() + Duration(seconds=timeout_s)

    while not received:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now() > deadline:
            node.destroy_subscription(sub)
            raise RuntimeError(
                f'Timed out ({timeout_s}s) waiting for pose on {topic}'
            )

    node.destroy_subscription(sub)
    return received[0]


def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, True)
    ])

    # Fetch dynamic start pose from procedural maze pipeline
    navigator.get_logger().info('Waiting for start pose from procedural pipeline ...')
    start_msg = _wait_for_pose(navigator, TOPIC_START, POSE_WAIT_TIMEOUT_S)
    navigator.get_logger().info(
        f'Start pose received: ({start_msg.pose.position.x:.3f}, '
        f'{start_msg.pose.position.y:.3f})'
    )

    # Fetch dynamic goal pose from procedural maze pipeline
    navigator.get_logger().info('Waiting for goal pose from procedural pipeline ...')
    goal_msg = _wait_for_pose(navigator, TOPIC_GOAL, POSE_WAIT_TIMEOUT_S)
    navigator.get_logger().info(
        f'Goal pose received: ({goal_msg.pose.position.x:.3f}, '
        f'{goal_msg.pose.position.y:.3f})'
    )

    # Setting the Initial Starting Position of our Robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose = start_msg.pose
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to fully activate (BasicNavigator retries internally)
    navigator.get_logger().info('Waiting for Nav2 to become active ...')
    navigator.waitUntilNav2Active()

    # Setting the Final Goal Position of our Robot
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose = goal_msg.pose
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()