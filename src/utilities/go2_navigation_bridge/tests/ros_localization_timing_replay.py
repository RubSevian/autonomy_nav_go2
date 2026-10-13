#!/usr/bin/env python3
"""Replay a local JSON/CDR raw-sensor capture through native LIO on localhost.

No SDK/robot controllers. Capture records: [relative_receive_s, cloud|imu,
base64_CDR]. Require ROS_DOMAIN_ID=228, ROS_LOCALHOST_ONLY=1 and loopback DDS.
"""
import argparse
import base64
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or 'name="lo"' not in os.environ.get('CYCLONEDDS_URI', '')):
        raise RuntimeError('Replay requires isolated domain228 and loopback DDS')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    records = json.loads(args.capture.read_text())
    if not records or len(records) > 20000:
        raise ValueError('Capture must contain 1..20000 records')
    previous = -1.
    for delay, kind, _ in records:
        if not math.isfinite(delay) or delay < previous or delay > 60 or kind not in ('imu', 'cloud'):
            raise ValueError('Invalid capture timing/stream')
        previous = delay
    profile = Path(__file__).resolve().parents[1] / 'config/real_navigation.yaml'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('localization_timing_replay')
    publishers = {k: node.create_publisher(t, '/utlidar/' + k, qos_profile_sensor_data)
                  for k, t in [('cloud', PointCloud2), ('imu', Imu)]}
    types = dict(cloud=PointCloud2, imu=Imu)
    stats, diagnostics, healthy = {}, {}, {}

    def receive(topic, msg):
        age = (node.get_clock().now().nanoseconds - msg.header.stamp.sec * 10**9 - msg.header.stamp.nanosec) / 1e9
        r = stats.setdefault(topic, dict(count=0, min_age_s=age, max_age_s=age, future_count=0))
        r['count'] += 1
        r['min_age_s'] = min(r['min_age_s'], age)
        r['max_age_s'] = max(r['max_age_s'], age)
        r['future_count'] += int(age < 0)

    def status(kind, msg):
        value = json.loads(msg.data)
        diagnostics[kind] = value
        if kind == 'nav' and value['tf_error'] == '' and all(
                value['health_ages'].get(k) is not None and value['health_ages'][k] <= .5
                for k in ('tf', 'imu', 'lidar', 'odom')):
            healthy.update(value)

    subscriptions = [node.create_subscription(t, topic, lambda m, k=topic: receive(k, m), qos_profile_sensor_data)
                     for topic, t in [('/utlidar/transformed_cloud', PointCloud2),
                                      ('/utlidar/transformed_raw_imu', Imu), ('/state_estimation', Odometry)]]
    subscriptions += [node.create_subscription(String, topic, lambda m, k=k: status(k, m), 1)
                      for k, topic in [('clock', '/utlidar/time_alignment'), ('nav', '/go2/navigation/status')]]
    process = None
    with args.output.with_suffix('.launch.log').open('w') as log:
        try:
            process = subprocess.Popen(['ros2', 'launch', 'go2_navigation_bridge', 'real_navigation.launch.py',
                                        'stage:=localization', 'calibration_path:=' + str(args.calibration),
                                        'profile_path:=' + str(profile)], stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            start = time.monotonic()
            while time.monotonic() - start < 3:
                assert process.poll() is None, 'localization launch exited'
                rclpy.spin_once(node, timeout_sec=.01)
            start = time.monotonic()
            for delay, kind, payload in records:
                while time.monotonic() - start < delay:
                    rclpy.spin_once(node, timeout_sec=.001)
                publishers[kind].publish(deserialize_message(base64.b64decode(payload), types[kind]))
                rclpy.spin_once(node, timeout_sec=0.)
            deadline = time.monotonic() + .15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            result = dict(topics=stats, diagnostics=diagnostics, healthy_nav=healthy,
                          lowcmd_publishers=node.count_publishers('/lowcmd'),
                          sport_publishers=node.count_publishers('/api/sport/request'))
            args.output.write_text(json.dumps(result, indent=2))
            assert len(stats) == 3 and all(r['count'] > 10 and r['future_count'] == 0 for r in stats.values()), result
            assert diagnostics['clock']['state'] == 'LOCKED', result
            assert healthy, result
            assert result['lowcmd_publishers'] == result['sport_publishers'] == 0, result
            print(json.dumps(result, indent=2))
            print('PASS real recorded raw → relay → native Point-LIO → odometry/TF/NAV health, without future stamps')
        finally:
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
