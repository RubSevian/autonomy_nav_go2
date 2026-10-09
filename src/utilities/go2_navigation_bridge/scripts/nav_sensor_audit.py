#!/usr/bin/env python3
"""Read-only rates/stamps/frames/TF audit. Publishes no velocity or robot commands."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--output',default='/home/ruben/go2_diploma/sim2real/runtime/nav_sensor_audit.json')
    args,ros_args=parser.parse_known_args()
    if not 0 < args.seconds <= 600:raise ValueError('seconds must be 0..600')
    rclpy.init(args=ros_args)
    node=Node('nav_sensor_audit')
    buffer=Buffer();listener=TransformListener(buffer,node)
    data={}
    def receive(topic,msg):
        row=data.setdefault(topic,dict(count=0,first=time.monotonic(),last=0.,frames=[],stamp_backwards=0,invalid=0,age_max_s=0.))
        row['count']+=1;row['last']=time.monotonic()
        if msg.header.frame_id not in row['frames']:row['frames'].append(msg.header.frame_id)
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp < row.get('previous_stamp',stamp):row['stamp_backwards']+=1
        row['previous_stamp']=stamp
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        row['age_max_s']=max(row['age_max_s'],age)
        if stamp<=0 or age<0 or age>.5 or not msg.header.frame_id:row['invalid']+=1
        if isinstance(msg,PointCloud2):
            row['fields']=[dict(name=f.name,datatype=f.datatype,offset=f.offset,count=f.count) for f in msg.fields]
            row['point_step']=msg.point_step;row['points']=msg.width*msg.height
        elif isinstance(msg,Imu):
            a,g=msg.linear_acceleration,msg.angular_velocity
            values=[a.x,a.y,a.z,g.x,g.y,g.z]
            if not all(math.isfinite(x) for x in values):
                row['invalid']+=1
                return
            row['last_acceleration']=values[:3];row['last_gyro']=values[3:]
        else:
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(x) for x in values):
                row['invalid']+=1
                return
            row.setdefault('first_position',values[:3]);row['last_position']=values[:3]
            row['child_frame_id']=msg.child_frame_id
    for topic,kind in [('/utlidar/cloud',PointCloud2),('/utlidar/imu',Imu),('/utlidar/transformed_cloud',PointCloud2),('/utlidar/transformed_raw_imu',Imu),('/state_estimation',Odometry)]:
        node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data)
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
        for row in data.values():
            row['rate_hz']=(row['count']-1)/(row['last']-row['first']) if row['count']>1 and row['last']>row['first'] else 0.
            row['receive_age_s']=time.monotonic()-row['last']
            if 'first_position' in row:row['position_displacement_m']=math.dist(row['first_position'],row['last_position'])
        tf={}
        for target,source in [('map','vehicle'),('map','camera_init'),('sensor','vehicle')]:
            try:
                t=buffer.lookup_transform(target,source,Time()).transform
                tf[target+'<-'+source]=dict(translation=[t.translation.x,t.translation.y,t.translation.z],quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            except Exception as e:tf[target+'<-'+source]=dict(unavailable=str(e))
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(dict(duration_s=args.seconds,topics=data,tf=tf,physical_extrinsics_validated=False,note='Displacement is drift only when operator confirms robot stationary; TF availability does not validate extrinsics.'),indent=2,allow_nan=False))
        print(output)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
