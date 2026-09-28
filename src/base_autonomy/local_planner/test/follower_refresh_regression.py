#!/usr/bin/env python3
"""Real follower process; synthetic inputs, isolated ROS domain (set externally)."""
import json
import os
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from visibility_graph_msg.msg import LocalPathConstraint


def run_case(model):
    child = subprocess.Popen([
        'ros2', 'run', 'local_planner', 'pathFollower', '--ros-args',
        '-p', 'enableNarrowPassageMode:=true', '-p', 'autonomyMode:=true',
        '-p', 'maxSpeed:=0.35', '-p', 'autonomySpeed:=0.35',
        '-p', 'maxAccel:=0.2', '-p', 'odomTimeoutSec:=2.0',
        '-p', 'pathTimeoutSec:=2.0', '-p', 'sendSportCommand:=false',
        '-p', 'followerMotionModel:=' + model], start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = rclpy.create_node('follower_refresh_regression')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, '/state_estimation', 10)
    path_pub = node.create_publisher(Path, '/path', 10)
    constraint_pub = node.create_publisher(LocalPathConstraint, '/local_path_constraint', qos)
    active_pub = node.create_publisher(Bool, '/navigation_active', qos)
    statuses = []
    commands = []
    node.create_subscription(DiagnosticArray, '/path_follower/status',
                             lambda msg: statuses.extend(msg.status), qos)
    node.create_subscription(TwistStamped, '/cmd_vel',
                             lambda msg: commands.append((time.monotonic(), msg.twist)), 10)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if child.poll() is not None:
                raise AssertionError('follower exited')
            rclpy.spin_once(node, timeout_sec=0.01)

    revision = 0
    def send(x, stop=False, repeat_active=False, path_first=False):
        nonlocal revision
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = x
        odom_pub.publish(odom)
        if repeat_active:
            active_pub.publish(Bool(data=True))
        spin(0.03)
        path = Path()
        path.header.stamp = node.get_clock().now().to_msg()
        path.header.frame_id = 'vehicle'
        for i in range(1 if stop else 41):
            pose = PoseStamped()
            pose.pose.position.x = i * 0.1
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        revision += 1
        constraint = LocalPathConstraint()
        constraint.header = path.header
        constraint.path_revision = revision
        constraint.candidate_group_id = -1 if stop else 3
        constraint.candidate_rotation_id = -1 if stop else 18
        constraint.candidate_direction = 0 if stop else 1
        constraint.switch_reason = 'NO_SAFE_CANDIDATE' if stop else 'REFRESH_SAME_CANDIDATE'
        if path_first:
            path_pub.publish(path)
            spin(0.03)
            constraint_pub.publish(constraint)
        else:
            constraint_pub.publish(constraint)
            spin(0.03)
            path_pub.publish(path)
        spin(0.1)

    try:
        until = time.monotonic() + 10
        while path_pub.get_subscription_count() == 0 and time.monotonic() < until:
            spin(0.05)
        assert path_pub.get_subscription_count(), 'DDS discovery failed'
        active_pub.publish(Bool(data=True))
        spin(0.1)
        for i in range(20):
            send(i * 0.1, path_first=(i % 2 == 1))
        values = {item.key: item.value for item in statuses[-1].values}
        assert int(values['tracking_point_index']) <= 6, values
        refreshes = int(values['path_refresh_count'])
        assert refreshes >= 18, values
        assert any(t.linear.x > 0.05 for _, t in commands), 'never moved'
        # A repeated active heartbeat must not destroy sticky candidate identity.
        send(2.0, repeat_active=True)
        values = {item.key: item.value for item in statuses[-1].values}
        assert int(values['path_refresh_count']) > refreshes, values
        assert int(values['path_switch_count']) == 0, values
        send(2.0, stop=True)
        assert statuses[-1].message == 'PLANNER_STOP_PATH', statuses[-1].message
        tail = [t for stamp, t in commands if stamp > commands[-1][0] - 0.04]
        assert tail and all(t.linear.x == 0 and t.linear.y == 0 and t.angular.z == 0
                            for t in tail), 'continued moving after planner stop'
        print(json.dumps({'model': model, 'result': 'PASS',
                          'tracking_point_index': values['tracking_point_index'],
                          'refresh_count': values['path_refresh_count'],
                          'switch_count': values['path_switch_count'],
                          'stop': statuses[-1].message}))
    finally:
        node.destroy_node()
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=5)


if __name__ == '__main__':
    rclpy.init()
    try:
        for model in ('holonomic', 'unicycle'):
            run_case(model)
    finally:
        rclpy.shutdown()
