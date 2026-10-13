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
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from navigation_profile import load_profile

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--output',default='/home/ruben/go2_diploma/sim2real/runtime/nav_sensor_audit.json')
    parser.add_argument('--profile-path',default=str(Path(get_package_share_directory('go2_navigation_bridge'))/'config/real_navigation.yaml'))
    args,ros_args=parser.parse_known_args()
    if not 0 < args.seconds <= 600:raise ValueError('seconds must be 0..600')
    profile=load_profile(args.profile_path)
    sensors=profile['sensors'];max_age=profile['bridge']['sensor_timeout_s']
    raw_topics={sensors['raw_cloud_topic'],sensors['raw_imu_topic']}
    rclpy.init(args=ros_args)
    node=Node('nav_sensor_audit')
    buffer=Buffer();listener=TransformListener(buffer,node)
    data={}
    diagnostics={}
    nav_health=dict(status_count=0, ready_count=0, observed_ready=False,
                    not_ready_after_first_ready=0, tf_error_after_first_ready=0,
                    previous_pair_count=0, selected_age_max_s=None,
                    position_error_max_m=None, quaternion_error_max=None,
                    odom_samples_max=0, tf_samples_max=0,
                    command_enabled_count=0, nonzero_command_count=0, failures=[])
    def diagnostic(name,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):return
            diagnostics[name]=value
            if name != 'navigation_status':return
            nav_health['status_count']+=1
            ready=value.get('navigation_ready') is True
            nav_health['ready_count']+=int(ready)
            nav_health['observed_ready'] |= ready
            nav_health['command_enabled_count']+=int(value.get('command_enabled') is True)
            nav_health['nonzero_command_count']+=int(any(value.get('command',[0,0,0])))
            pair=value.get('tf_pair',{})
            nav_health['previous_pair_count']+=int(pair.get('using_previous_pair') is True)
            for key,source in [('selected_age_max_s','selected_age_s'),
                               ('position_error_max_m','position_error_m'),
                               ('quaternion_error_max','quaternion_error')]:
                number=pair.get(source)
                if type(number) in (int,float) and math.isfinite(number):
                    nav_health[key]=number if nav_health[key] is None else max(nav_health[key],number)
            for key,source in [('odom_samples_max','odom_samples'),('tf_samples_max','tf_samples')]:
                nav_health[key]=max(nav_health[key],pair.get(source,0))
            if nav_health['observed_ready']:
                nav_health['not_ready_after_first_ready']+=int(not ready)
                nav_health['tf_error_after_first_ready']+=int(bool(value.get('tf_error')))
                if (not ready or value.get('tf_error')) and len(nav_health['failures']) < 20:
                    nav_health['failures'].append(dict(elapsed_s=time.monotonic()-start,
                        tf_error=value.get('tf_error'), block_reason=value.get('block_reason'),
                        tf_pair=pair, health_ages=value.get('health_ages')))
        except ValueError:pass
    node.create_subscription(String,sensors['time_status_topic'],lambda m:diagnostic('time_alignment',m),1)
    node.create_subscription(String,'/go2/navigation/status',lambda m:diagnostic('navigation_status',m),1)
    def receive(topic,msg):
        raw=topic in raw_topics
        row=data.setdefault(topic,dict(count=0,first=time.monotonic(),last=0.,frames=[],stamp_backwards=0,invalid=0,
                                       header_clock='robot_unsynchronized' if raw else 'ros',
                                       age_min_s=None,age_max_s=None,future_count=0,stale_count=0))
        row['count']+=1;row['last']=time.monotonic()
        if msg.header.frame_id not in row['frames']:row['frames'].append(msg.header.frame_id)
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp < row.get('previous_stamp',stamp):row['stamp_backwards']+=1
        row['previous_stamp']=stamp
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        prefix='clock_delta_to_ros' if raw else 'age'
        for suffix,func in (('min_s',min),('max_s',max)):
            key=prefix+'_'+suffix
            row[key]=age if row.get(key) is None else func(row[key],age)
        # Raw stamps belong to the unsynchronized robot clock, not ROS time.
        if not raw:
            row['future_count']+=int(age<0)
            row['stale_count']+=int(age>max_age)
        if stamp<=0 or (not raw and (age<0 or age>max_age)) or not msg.header.frame_id:row['invalid']+=1
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
    for topic,kind in [(sensors['raw_cloud_topic'],PointCloud2),(sensors['raw_imu_topic'],Imu),
                       (sensors['cloud_topic'],PointCloud2),(sensors['raw_imu_output_topic'],Imu),('/state_estimation',Odometry)]:
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
        output.write_text(json.dumps(dict(duration_s=args.seconds,topics=data,tf=tf,**diagnostics,navigation_health=nav_health,physical_extrinsics_validated=False,
                                         note='Raw clock deltas are not packet ages. Displacement is drift only when operator confirms robot stationary; TF availability does not validate extrinsics.'),indent=2,allow_nan=False))
        print(output)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
