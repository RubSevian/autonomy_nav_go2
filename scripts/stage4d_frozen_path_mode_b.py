#!/usr/bin/env python3
"""Mode B: capture one planner path, then replay it with exclusive /path ownership."""
import argparse
import copy
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from visibility_graph_msg.msg import LocalPathConstraint

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR_OFFSET_X = -0.02557


class FrozenPath(Node):
    def __init__(self):
        super().__init__('stage4d_frozen_path_mode_b')
        self.path = None
        self.constraint = None
        self.odom = None
        self.world_points = None
        self.create_subscription(Path, '/path', self.on_path, 10)
        self.create_subscription(LocalPathConstraint, '/local_path_constraint',
                                 self.on_constraint, QOS)
        self.create_subscription(Odometry, '/state_estimation', self.on_odom, 10)

    def on_path(self, msg):
        if len(msg.poses) > 1 and msg.header.frame_id in ('vehicle', '/vehicle'):
            self.path = copy.deepcopy(msg)

    def on_constraint(self, msg):
        self.constraint = copy.deepcopy(msg)

    def on_odom(self, msg):
        self.odom = msg

    def pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.x - math.cos(yaw) * SENSOR_OFFSET_X,
                p.y - math.sin(yaw) * SENSOR_OFFSET_X, yaw)

    def ready(self):
        return (self.path is not None and self.constraint is not None
                and self.pose() is not None
                and self.path.header.stamp == self.constraint.header.stamp)

    def capture(self):
        x, y, yaw = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        self.world_points = [
            (x + c * pose.pose.position.x - s * pose.pose.position.y,
             y + s * pose.pose.position.x + c * pose.pose.position.y)
            for pose in self.path.poses
        ]
        return {
            'size': len(self.world_points),
            'group_id': self.constraint.candidate_group_id,
            'rotation_id': self.constraint.candidate_rotation_id,
            'direction': self.constraint.candidate_direction,
            'requires_alignment': self.constraint.requires_alignment,
        }

    def publish(self, path_pub, constraint_pub):
        pose = self.pose()
        if pose is None:
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        path = copy.deepcopy(self.path)
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp
        for item, (world_x, world_y) in zip(path.poses, self.world_points):
            dx, dy = world_x - x, world_y - y
            item.pose.position.x = c * dx + s * dy
            item.pose.position.y = -s * dx + c * dy
        constraint = copy.deepcopy(self.constraint)
        constraint.header.stamp = stamp
        constraint.switch_reason = 'MODE_B_FROZEN_PLANNER_PATH'
        constraint_pub.publish(constraint)
        path_pub.publish(path)


def find_owned_planner(pgid):
    result = subprocess.run(['ps', '-eo', 'pid=,pgid=,args='], check=True,
                            capture_output=True, text=True)
    matches = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=2)
        if len(fields) == 3 and int(fields[1]) == pgid and (
                '/lib/local_planner/localPlanner ' in fields[2]):
            matches.append(int(fields[0]))
    if len(matches) != 1:
        raise RuntimeError(f'expected exactly one owned localPlanner, got {matches}')
    return matches[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch-pgid', type=int, required=True)
    parser.add_argument('--duration', type=float, default=25.0)
    parser.add_argument('--capture-timeout', type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = FrozenPath()
    try:
        deadline = time.monotonic() + args.capture_timeout
        while rclpy.ok() and time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.ready():
            raise RuntimeError('no stamped planner path/constraint pair or odometry')
        print(f'CAPTURE {node.capture()}', flush=True)
        planner_pid = find_owned_planner(args.launch_pgid)
        os.kill(planner_pid, signal.SIGINT)
        grace = time.monotonic() + 2.0
        while time.monotonic() < grace and os.path.exists(f'/proc/{planner_pid}'):
            rclpy.spin_once(node, timeout_sec=0.05)
        if os.path.exists(f'/proc/{planner_pid}'):
            os.kill(planner_pid, signal.SIGKILL)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.count_publishers('/path') == 0:
                break
        if node.count_publishers('/path') != 0:
            raise RuntimeError('localPlanner still owns /path; refusing mixed sources')
        path_pub = node.create_publisher(Path, '/path', QOS)
        constraint_pub = node.create_publisher(LocalPathConstraint,
                                               '/local_path_constraint', QOS)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            if node.count_publishers('/path') != 1:
                raise RuntimeError('duplicate /path publisher during frozen run')
            node.publish(path_pub, constraint_pub)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
