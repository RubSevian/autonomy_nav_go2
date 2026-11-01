#!/usr/bin/env python3
"""Actual installed follower, synthetic paired paths; localhost only, no actuator."""
import math
import os
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Requires localhost ROS_DOMAIN_ID=228')

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from visibility_graph_msg.msg import LocalPathConstraint
from diagnostic_msgs.msg import DiagnosticArray


def main():
    rclpy.init()
    node = rclpy.create_node('follower_reference_smoke')
    latched = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, '/state_estimation', 5)
    path_pub = node.create_publisher(Path, '/path', 5)
    constraint_pub = node.create_publisher(LocalPathConstraint, '/local_path_constraint', latched)
    active_pub = node.create_publisher(Bool, '/navigation_active', latched)
    commands = []
    statuses = []
    node.create_subscription(DiagnosticArray, '/path_follower/status',
                             lambda m: statuses.extend(m.status), latched)
    node.create_subscription(TwistStamped, '/cmd_vel', lambda m: commands.append(
        (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)), 5)
    command = ['ros2', 'run', 'local_planner', 'pathFollower', '--ros-args']
    for key, value in dict(autonomyMode=True, autonomySpeed=.2, maxSpeed=.2,
                           maxAccel=1., sensorOffsetX=0., sensorOffsetY=0.,
                           odomTimeoutSec=.5, pathTimeoutSec=.5,
                           allowStaticPath=False).items():
        command += ['-p', f'{key}:={str(value).lower()}']
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, start_new_session=True)
    revision = 0

    def pump(seconds, source_x=0., yaw=0., source_age=.1, odom_age=0., paired=True):
        nonlocal revision
        commands.clear()
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            assert process.poll() is None, 'follower exited'
            now = node.get_clock().now()
            stamp = now.to_msg()
            odom = Odometry()
            odom.header.frame_id = 'camera_init'
            odom.header.stamp = (now-rclpy.duration.Duration(seconds=odom_age)).to_msg()
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
            active_pub.publish(Bool(data=True))
            path = Path()
            path.header.frame_id = 'vehicle'
            path.header.stamp = stamp
            for i in range(21):
                point = PoseStamped()
                point.pose.position.x = i / 20.
                point.pose.orientation.w = 1.
                path.poses.append(point)
            constraint = LocalPathConstraint()
            constraint.header = path.header
            revision += 1
            constraint.path_revision = revision
            constraint.candidate_group_id = 1
            constraint.candidate_rotation_id = 0
            constraint.candidate_direction = 1
            constraint.speed_scale = 1.
            constraint.source_pose.header.frame_id = 'camera_init'
            constraint.source_pose.header.stamp = (now-rclpy.duration.Duration(seconds=source_age)).to_msg()
            constraint.source_pose.pose.position.x = source_x
            constraint.source_pose.pose.orientation.z = math.sin(yaw/2)
            constraint.source_pose.pose.orientation.w = math.cos(yaw/2)
            # Exercise both delivery orders.
            if revision % 2:
                path_pub.publish(path)
                if paired:
                    constraint_pub.publish(constraint)
            else:
                if paired:
                    constraint_pub.publish(constraint)
                path_pub.publish(path)
            # Fixed 20 Hz fixture. spin_once may return immediately on a ready
            # command; using it as a timer would flood the paired route queues.
            cycle_end = time.monotonic() + .05
            while time.monotonic() < cycle_end:
                rclpy.spin_once(node, timeout_sec=.005)
        assert commands, 'no command observations'

    def stopped():
        assert all(max(map(abs, v)) < 1.e-6 for v in commands[-5:]), commands[-5:]

    try:
        pump(3.)
        assert any(v[0] > .05 for v in commands[-10:]), (commands[-10:], statuses[-1:])
        # Target is already at current position. Reanchoring to receipt pose
        # would wrongly place it one metre ahead and continue motion.
        pump(1., source_x=-1.)
        stopped()
        pump(1., yaw=.4)
        assert any(v[1] > .02 for v in commands[-10:]), commands[-10:]
        pump(1., yaw=-.4)
        assert any(v[1] < -.02 for v in commands[-10:]), commands[-10:]
        pump(1., source_age=1.)
        stopped()
        pump(1.)
        assert any(v[0] > .05 for v in commands[-10:])
        pump(1., odom_age=1.)
        stopped()
        pump(1., paired=False)
        stopped()
        assert not node.get_publishers_info_by_topic('/lowcmd')
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        print('PASS actual follower: delayed origin, rotated refresh, both pairing orders, stale source/odometry, missing metadata; no actuator publishers')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
