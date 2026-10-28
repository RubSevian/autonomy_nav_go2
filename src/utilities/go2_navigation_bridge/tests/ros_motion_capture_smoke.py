#!/usr/bin/env python3
"""Synthetic recorder integration on domain173/loopback; no robot nodes or SDK."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='173' or 'name="lo"' not in os.environ.get('CYCLONEDDS_URI',''):
        raise RuntimeError('Requires isolated ROS domain173 and loopback DDS')
    package=Path(__file__).resolve().parents[1]
    rclpy.init();node=rclpy.create_node('synthetic_motion_capture_source')
    odom=node.create_publisher(Odometry,'/state_estimation',qos_profile_sensor_data)
    velocity=node.create_publisher(TwistWithCovarianceStamped,'/point_lio/velocity',qos_profile_sensor_data)
    command=node.create_publisher(TwistStamped,'/cmd_vel',10)
    mission=node.create_publisher(String,'/go2/mission/status',10)
    navigation=node.create_publisher(String,'/go2/navigation/status',10)
    try:
        with tempfile.TemporaryDirectory(prefix='motion-capture-smoke-') as directory:
            root=Path(directory);capture=root/'capture'
            with (root/'recorder.log').open('w+') as log:
                process=subprocess.Popen([sys.executable,str(package/'scripts/nav_drift.py'),
                    '--config',str(package/'config/real_navigation.yaml'),'--output',str(capture),'--until-shutdown'],
                    stdout=log,stderr=subprocess.STDOUT)
                try:
                    begin=time.monotonic();scan=None;home=None
                    while time.monotonic()-begin<7.:
                        assert process.poll() is None,'recorder exited early'
                        elapsed=time.monotonic()-begin;mono=time.monotonic_ns();stamp=node.get_clock().now().to_msg()
                        o=Odometry();o.header.stamp=stamp;o.header.frame_id='camera_init';o.child_frame_id='aft_mapped'
                        o.pose.pose.orientation.w=1.;o.pose.pose.position.x=.3+.2*elapsed;odom.publish(o)
                        v=TwistWithCovarianceStamped();v.header.stamp=stamp;v.header.frame_id='aft_mapped'
                        v.twist.twist.linear.x=.2;v.twist.covariance=[.0001 if i%7==0 else 0. for i in range(36)];velocity.publish(v)
                        c=TwistStamped();c.header.stamp=stamp;c.header.frame_id='vehicle';c.twist.linear.x=.15;command.publish(c)
                        if elapsed>=2 and scan is None:scan=mono
                        if elapsed>=5 and home is None:home=mono
                        state='GRASP_PRESUMED_SUCCESS' if home else 'ARM_SCAN_POSE' if scan else 'NAVIGATING'
                        mission.publish(String(data=json.dumps(dict(observed_ns=mono,session_id='synthetic-mission',
                            navigation_session_id='synthetic-nav',goal_revision=1,state=state,grasp_assessment='GRASPED' if home else 'UNKNOWN',
                            base_motion_window=dict(scan_started_ns=scan,home_completed_ns=home)))))
                        navigation.publish(String(data=json.dumps(dict(observed_ns=mono,session_id='synthetic-nav'))))
                        rclpy.spin_once(node,timeout_sec=.02)
                    process.send_signal(signal.SIGINT);process.wait(timeout=20)
                    log.seek(0);text=log.read();assert process.returncode==0,text
                    info=json.loads((capture/'capture.json').read_text())
                    result=json.loads((capture/'base_motion_results.json').read_text())
                    assert info['own_publishers']==[] and info['own_clients']==[],info
                    assert info['velocity_samples']>100 and info['until_shutdown'],info
                    assert result['velocity_status']=='EKF_RECORDED',result
                    window=result['manipulation_windows'][0]
                    assert window['metrics']['status']=='RELATIVE_ODOMETRY_ONLY',window
                    assert .5<window['metrics']['displacement_xy_m']<.7,window
                    error=result['navigation_comparison']['before_grasp']['command_tracking_rmse_xy_mps']
                    assert error is not None and abs(error-.05)<1e-6,result
                    assert node.count_publishers('/lowcmd')==0 and node.count_publishers('/api/sport/request')==0
                    print(json.dumps(dict(status='PASS',poses=info['poses'],velocities=info['velocity_samples'],
                        manipulation_displacement_m=window['metrics']['displacement_xy_m'],tracking_rmse_mps=error,
                        recorder_publishers=info['own_publishers'],recorder_clients=info['own_clients'])))
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:process.wait(timeout=20)
                        except subprocess.TimeoutExpired:process.kill();process.wait()
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
