#!/usr/bin/env python3
"""Isolated localPlanner/pathFollower contract smoke test; no FAR or MuJoCo claim."""
import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from visibility_graph_msg.msg import LocalPathConstraint


class Probe(Node):
    def __init__(self, gap, open_space):
        super().__init__('narrow_contract_probe')
        self.gap = gap
        self.open_space = open_space
        self.active_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/terrain_map', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.active_pub = self.create_publisher(Bool, '/navigation_active', self.active_qos)
        self.constraints = []
        self.states = set()
        self.max_vx = 0.0
        self.max_vy = 0.0
        self.path_sizes = []
        self.cycle_ms = []
        self.narrow_ms = []
        self.local_recovered = 0
        self.local_counters = {}
        self.selection = {}
        self.follower_counters = {}
        self.create_subscription(LocalPathConstraint, '/local_path_constraint',
                                 self.constraint_cb, self.active_qos)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(DiagnosticArray, '/local_planner/status', self.local_cb,
                                 self.active_qos)
        self.create_subscription(DiagnosticArray, '/path_follower/status', self.follower_cb,
                                 self.active_qos)

    def constraint_cb(self, msg):
        self.constraints.append((msg.path_revision, msg.requires_alignment,
                                 msg.header.stamp.sec, msg.header.stamp.nanosec))

    def path_cb(self, msg):
        self.path_sizes.append(len(msg.poses))

    def cmd_cb(self, msg):
        self.max_vx = max(self.max_vx, msg.twist.linear.x)
        self.max_vy = max(self.max_vy, abs(msg.twist.linear.y))

    def local_cb(self, msg):
        for status in msg.status:
            if status.name != 'local_planner':
                continue
            values = {item.key: item.value for item in status.values}
            for key, target in [('planning_cycle_ms', self.cycle_ms),
                                ('narrow_check_ms', self.narrow_ms)]:
                try:
                    value = float(values.get(key, 'nan'))
                    if math.isfinite(value) and value > 0:
                        target.append(value)
                except ValueError:
                    pass
            self.local_recovered = max(
                self.local_recovered,
                int(values.get('narrow_phase_recovered_candidates', '0')))
            for key in ('final_selectable_count', 'recovered_entered_selection',
                        'candidate_paths_eligible', 'broad_phase_blocked_candidates',
                        'narrow_phase_checked_candidates',
                        'narrow_phase_recovered_candidates',
                        'narrow_phase_rejected_candidates'):
                if key in values and values[key] != 'N/A':
                    self.local_counters[key] = max(
                        self.local_counters.get(key, 0), int(values[key]))

            for key in ('best_candidate_id', 'best_recovered_candidate_id',
                        'selection_failure_reason', 'path_found',
                        'published_path_size', 'selected_group_id'):
                if key in values:
                    self.selection[key] = values[key]
    def follower_cb(self, msg):
        for status in msg.status:
            if status.name == 'path_follower':
                values = {item.key: item.value for item in status.values}
                self.states.add(values.get('narrow_state', 'N/A'))
                for key in ('alignment_entries', 'alignment_successes',
                            'realign_events', 'alignment_timeouts'):
                    if key in values:
                        self.follower_counters[key] = max(
                            self.follower_counters.get(key, 0), int(values[key]))
                if 'max_narrow_yaw_error' in values:
                    self.follower_counters['max_narrow_yaw_error'] = max(
                        self.follower_counters.get('max_narrow_yaw_error', 0.0),
                        float(values['max_narrow_yaw_error']))

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        cloud_header = odom.header
        fields = [PointField(name=name, offset=4 * index,
                             datatype=PointField.FLOAT32, count=1)
                  for index, name in enumerate(('x', 'y', 'z', 'intensity'))]
        # Two parallel obstacle rows at a configurable physical gap.
        points = [] if self.open_space else [
            (0.4 + index * 0.1, side * self.gap / 2.0, 0.0, 0.5)
            for index in range(13) for side in (-1, 1)]
        self.cloud_pub.publish(point_cloud2.create_cloud(cloud_header, fields, points))
        goal = PointStamped()
        goal.header = odom.header
        goal.point.x = 2.0
        self.goal_pub.publish(goal)
        self.active_pub.publish(Bool(data=True))


def stats(values):
    if not values:
        return None
    data = sorted(values)
    return {'mean': round(statistics.mean(data), 3),
            'p95': round(data[min(len(data) - 1, int(0.95 * len(data)))], 3),
            'max': round(data[-1], 3), 'samples': len(data)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--enable', action='store_true')
    parser.add_argument('--gap', type=float, default=0.66)
    parser.add_argument('--open-space', action='store_true')
    parser.add_argument('--expect-reject', action='store_true')
    parser.add_argument('--duration', type=float, default=12.0)
    parser.add_argument('--path-folder', required=True)
    args = parser.parse_args()
    mode = 'true' if args.enable else 'false'
    subprocesses = []
    env = os.environ.copy()
    try:
        local = ['ros2', 'run', 'local_planner', 'localPlanner', '--ros-args',
                 '-p', 'pathFolder:=' + args.path_folder,
                 '-p', 'enableNarrowPassageMode:=' + mode,
                 '-p', 'useTerrainAnalysis:=true', '-p', 'obstacleHeightThre:=0.3',
                 '-p', 'pathScale:=0.75', '-p', 'minPathScale:=0.75',
                 '-p', 'adjacentRange:=3.0', '-p', 'autonomyMode:=true',
                 '-p', 'autonomySpeed:=0.35', '-p', 'maxSpeed:=0.35',
                 '-p', 'goalCloseDis:=0.3']
        follower = ['ros2', 'run', 'local_planner', 'pathFollower', '--ros-args',
                    '-p', 'enableNarrowPassageMode:=' + mode,
                    '-p', 'autonomyMode:=true', '-p', 'autonomySpeed:=0.35',
                    '-p', 'maxSpeed:=0.35', '-p', 'sendSportCommand:=false']
        for argv in (local, follower):
            subprocesses.append(subprocess.Popen(
                argv, env=env, start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        rclpy.init()
        probe = Probe(args.gap, args.open_space)
        until = time.monotonic() + args.duration
        next_publish = 0.0
        while time.monotonic() < until:
            if any(child.poll() is not None for child in subprocesses):
                raise RuntimeError('planner or follower exited unexpectedly')
            if time.monotonic() >= next_publish:
                probe.publish_inputs()
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(probe, timeout_sec=0.01)
        outcome = {'mode': mode, 'gap': args.gap, 'open_space': args.open_space,
                   'constraint_count': len(probe.constraints),
                   'requires_alignment': any(item[1] for item in probe.constraints),
                   'follower_states': sorted(probe.states),
                   'max_vx': round(probe.max_vx, 3),
                   'max_abs_vy': round(probe.max_vy, 3),
                   'path_sizes': sorted(set(probe.path_sizes)),
                   'recovered_candidates': probe.local_recovered,
                   'local_counters': probe.local_counters,
                   'follower_counters': probe.follower_counters,
                   'selection': probe.selection,
                   'cycle_ms': stats(probe.cycle_ms),
                   'narrow_check_ms': stats(probe.narrow_ms)}
        print(json.dumps(outcome, sort_keys=True))
        probe.destroy_node()
        rclpy.shutdown()
        if args.enable and not args.open_space and not args.expect_reject and not (outcome['requires_alignment'] and
                                'NARROW_TRAVERSE' in probe.states and
                                outcome['max_vx'] > 0.1 and
                                outcome['max_abs_vy'] < 1.0e-6 and
                                int(outcome['local_counters'].get('final_selectable_count', 0)) > 0 and
                                int(outcome['local_counters'].get('recovered_entered_selection', 0)) > 0 and
                                outcome['selection'].get('path_found') == 'true' and
                                int(outcome['selection'].get('published_path_size', 0)) > 1):
            return 2
        if (not args.enable or args.open_space or args.expect_reject) and outcome['requires_alignment']:
            return 2
        return 0
    finally:
        for child in subprocesses:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in subprocesses:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
