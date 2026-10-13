#!/usr/bin/env python3
"""Actual sensor relay, synthetic best-effort inputs only; no robot/SDK nodes."""
import os
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import sys
import yaml
if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Synthetic test requires localhost ROS_DOMAIN_ID=228')
import rclpy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_contract import valid_cloud_layout

rclpy.init()
node=rclpy.create_node('sensor_wiring_smoke')
profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/real_navigation.yaml').read_text())
received={key:[] for key in ('cloud','imu','legacy')}
for key,topic,kind in [('cloud',profile['sensors']['cloud_topic'],PointCloud2),
                       ('imu',profile['sensors']['raw_imu_output_topic'],Imu),
                       ('legacy',profile['sensors']['legacy_imu_output_topic'],Imu)]:
    node.create_subscription(kind,topic,lambda m,k=key:received[k].append(m),10)
cloud_pub=node.create_publisher(PointCloud2,profile['sensors']['raw_cloud_topic'],qos_profile_sensor_data)
imu_pub=node.create_publisher(Imu,profile['sensors']['raw_imu_topic'],qos_profile_sensor_data)
process=None
with tempfile.TemporaryDirectory(prefix='nav-sensor-fixture-') as directory:
    calibration=Path(directory)/'calibration.yaml'
    calibration.write_text(''.join(key+': 0.0\n' for key in ('acc_bias_x','acc_bias_y','acc_bias_z','ang_bias_x','ang_bias_y','ang_bias_z','ang_z2x_proj','ang_z2y_proj')))
    params=Path(directory)/'params.yaml'
    params.write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(profile['sensors'],calibration_path=str(calibration))}}))
    log=(Path(directory)/'relay.log').open('w+')
    try:
        process=subprocess.Popen(['ros2','run','transform_sensors','transform_everything','--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        fields=[PointField(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in
                [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('ring',20,4),('time',24,7)]]
        started=time.monotonic()
        sensor_start=started
        while time.monotonic()-started<8 and not all(received.values()):
            assert process.poll() is None,'relay exited'
            stamp=Time(nanoseconds=100_000_000_000+int((time.monotonic()-sensor_start)*1e9)).to_msg()
            header=Header(stamp=stamp,frame_id='utlidar_lidar_1')
            cloud=point_cloud2.create_cloud(header,fields,[(1.,.2,.1,10.,3,.001),(2.,.3,.2,20.,17,.002)],point_step=32)
            cloud_pub.publish(cloud)
            imu=Imu();imu.header=Header(stamp=stamp,frame_id='utlidar_imu_1');imu.orientation.w=1.
            imu.linear_acceleration.z=-9.81;imu.angular_velocity.z=.1
            imu_pub.publish(imu)
            rclpy.spin_once(node,timeout_sec=.02)
        assert all(received.values()),{key:len(v) for key,v in received.items()}
        cloud=received['cloud'][-1];imu=received['imu'][-1];legacy=received['legacy'][-1]
        assert cloud.header.frame_id==imu.header.frame_id==legacy.header.frame_id=='body'
        now_ns=node.get_clock().now().nanoseconds
        cloud_stamp=cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
        imu_stamp=imu.header.stamp.sec*10**9+imu.header.stamp.nanosec
        assert 0 <= now_ns-cloud_stamp < 500_000_000
        assert 0 <= now_ns-imu_stamp < 500_000_000
        assert valid_cloud_layout(cloud)
        assert cloud.point_step==32 and cloud.row_step==cloud.width*32
        points=point_cloud2.read_points(cloud)
        assert list(points['ring'])==[3,17]
        assert all(math.isclose(a,b,rel_tol=1e-6) for a,b in zip(points['time'],(.001,.002)))
        assert [(f.name,f.datatype,f.offset) for f in cloud.fields]==[(f.name,f.datatype,f.offset) for f in fields]
        assert math.isclose(math.sqrt(sum(v*v for v in (imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z))),9.81,rel_tol=1e-6)
        assert (legacy.linear_acceleration.x,legacy.linear_acceleration.y,legacy.linear_acceleration.z)==(0.,0.,0.)
        received['cloud'].clear()
        started=time.monotonic()
        while time.monotonic()-started<3 and not any(m.width==0 for m in received['cloud']):
            assert process.poll() is None,'relay exited on empty padded cloud'
            header=Header(stamp=Time(nanoseconds=100_000_000_000+int((time.monotonic()-sensor_start)*1e9)).to_msg(),frame_id='utlidar_lidar_1')
            cloud_pub.publish(point_cloud2.create_cloud(header,fields,[],point_step=32))
            rclpy.spin_once(node,timeout_sec=.02)
        empty=next(m for m in received['cloud'] if m.width==0)
        assert empty.point_step==32 and empty.row_step==0 and empty.header.frame_id=='body'
        assert not node.get_publishers_info_by_topic('/lowcmd')
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        print('PASS actual relay with best-effort raw inputs: Unitree padded 32-byte records and empty clouds preserved; body frames, float32 time, uint16 ring; Point-LIO acceleration retained, legacy acceleration zero')
    finally:
        if process and process.poll() is None:process.send_signal(signal.SIGINT)
        if process:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.seek(0);output=log.read();log.close()
        assert 'Traceback' not in output,output
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
