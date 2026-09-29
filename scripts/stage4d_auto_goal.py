#!/usr/bin/env python3
"""Publish exactly one Stage4D goal only after FAR reports V-Graph readiness."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TRANSIENT = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AutoGoal(Node):
    def __init__(self, goal_x, goal_y, timeout, output):
        super().__init__('stage4d_auto_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.deadline = time.monotonic() + timeout
        self.output = Path(output)
        self.state = 'WAITING_FOR_FAR_READY'
        self.publish_timestamp = None
        self.ack_timestamp = None
        self.error = None
        self.done = False
        self.transitions = []
        self.publisher = self.create_publisher(PointStamped, '/goal_point', 1)
        self.create_subscription(DiagnosticArray, '/far/planner_status',
                                 self.on_far_status, TRANSIENT)
        self.timer = self.create_timer(0.05, self.tick)
        self.record()

    def record(self):
        self.transitions.append({'state': self.state, 'timestamp': time.time()})
        data = {
            'auto_goal_status': self.state,
            'auto_goal_publish_timestamp': self.publish_timestamp,
            'auto_goal_ack_timestamp': self.ack_timestamp,
            'error': self.error,
            'goal': {'frame_id': 'map', 'x': self.goal_x, 'y': self.goal_y},
            'transitions': self.transitions,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(data, indent=2) + '\n')
        print(json.dumps(data, sort_keys=True), flush=True)

    @staticmethod
    def state_from(msg):
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            yield values.get('state', status.message), values.get('reason_code', '')

    def on_far_status(self, msg):
        for state, _ in self.state_from(msg):
            if self.state == 'WAITING_FOR_FAR_READY' and state == 'VGRAPH_READY':
                self.state = 'READY_OBSERVED'
                self.record()
                self.publish_goal()
            if self.state == 'GOAL_PUBLISHED' and state in (
                    'GOAL_RECEIVED', 'GOAL_ACCEPTED', 'WAYPOINT_PUBLISHED', 'GOAL_PENDING'):
                self.state = 'GOAL_ACKNOWLEDGED'
                self.ack_timestamp = time.time()
                self.record()
                self.state = 'DONE'
                self.done = True
                self.record()

    def publish_goal(self):
        if self.state != 'READY_OBSERVED':
            return
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.point.x = self.goal_x
        goal.point.y = self.goal_y
        self.publisher.publish(goal)
        self.publish_timestamp = time.time()
        self.state = 'GOAL_PUBLISHED'
        self.record()

    def tick(self):
        if self.done:
            return
        if self.state == 'READY_OBSERVED':
            self.publish_goal()
        if time.monotonic() >= self.deadline:
            self.error = ('FAR readiness was not observed' if
                      self.state == 'WAITING_FOR_FAR_READY' else
                       'published D0 was not acknowledged by FAR')
            self.state = 'FAILED'
            self.done = True
            self.record()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=1.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = AutoGoal(args.x, args.y, args.timeout, args.output)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        success = node.state == 'DONE' and node.error is None
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
