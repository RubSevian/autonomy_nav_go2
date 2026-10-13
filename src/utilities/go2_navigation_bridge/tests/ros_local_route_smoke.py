#!/usr/bin/env python3
"""Installed localPlanner + follower + bridge; fake sensors/FSM, no robot owner."""
import json
import os
from pathlib import Path
import signal
import struct
import csv
import uuid
import subprocess
import time
import sys
if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Synthetic test requires localhost ROS_DOMAIN_ID=228')
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

rclpy.init()
node=rclpy.create_node('local_route_smoke')
statuses=[]
node.create_subscription(String,'/go2/navigation/status',lambda m:statuses.append(json.loads(m.data)),1)
# Bound test observations rather than retaining long sensor recordings.
commands=[]
from geometry_msgs.msg import TwistStamped
node.create_subscription(TwistStamped,'/cmd_vel',lambda m:commands.append((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),1)
pubs={name:node.create_publisher(typ,topic,1) for name,typ,topic in (
    ('fsm',String,'/go2/locomotion_status'),('cloud',PointCloud2,'/utlidar/transformed_cloud'),
    ('imu',Imu,'/utlidar/transformed_raw_imu'),('odom',Odometry,'/state_estimation'),
    ('terrain',PointCloud2,'/terrain_map'),('goal',PointStamped,'/way_point'))}
static=StaticTransformBroadcaster(node)
tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='camera_init';tf.transform.rotation.w=1.
base=TransformStamped();base.header.frame_id='aft_mapped';base.child_frame_id='vehicle';base.transform.rotation.w=1.
static.sendTransform([tf,base]);dynamic=TransformBroadcaster(node)
root=Path(__file__).resolve().parents[6]
paths=str(root/'install_nav_stack/local_planner/share/local_planner/paths')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_profile import load_profile, component_parameters
profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
# Synthetic TF fixture places odometry and vehicle at the same origin.
profile['robot']['sensor_offset_x_m']=0.
profile['robot']['sensor_offset_y_m']=0.
components=component_parameters(profile)
components['localPlanner']['pathFolder']=paths
processes=[]
logs=[]
try:
    for package,executable,params in (
        ('go2_navigation_bridge','go2_navigation_bridge.py',dict(profile['bridge'],goal_tolerance_m=profile['local']['goal_tolerance_m'],input_mode='stack',local_route=True)),
        ('local_planner','localPlanner',components['localPlanner']),
        ('local_planner','pathFollower',components['pathFollower'])):
        command=['ros2','run',package,executable,'--ros-args']
        for key,value in params.items():command+=['-p',key+':='+str(value).lower() if type(value) is bool else key+':='+str(value)]
        if package=='local_planner':command+=['-r','/cmd_vel:=/navigation/cmd_vel']
        log=open(root/'runtime'/('nav_smoke_'+executable+'.log'),'w');logs.append(log)
        processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    x=0.
    def pump(seconds,cloud=True,blocked=False):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            assert all(p.poll() is None for p in processes[:3]),'NAV process exited'
            stamp=node.get_clock().now().to_msg()
            pubs['fsm'].publish(String(data=json.dumps(dict(state='ACTIVE',phase='RL_ZERO',output_enabled=True,fault_latched=False,command_source='NAVIGATION'))))
            odom=Odometry();odom.header.frame_id='camera_init';odom.header.stamp=stamp;odom.child_frame_id='aft_mapped';odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=x;pubs['odom'].publish(odom)
            tf=TransformStamped();tf.header.frame_id='camera_init';tf.child_frame_id='aft_mapped';tf.header.stamp=stamp;tf.transform.rotation.w=1.;tf.transform.translation.x=x;dynamic.sendTransform(tf)
            imu=Imu();imu.header.frame_id='body';imu.header.stamp=stamp;imu.linear_acceleration.z=9.81;pubs['imu'].publish(imu)
            pc=PointCloud2();pc.header.frame_id='body';pc.header.stamp=stamp;pc.width=1;pc.height=1;pc.point_step=20;pc.row_step=20;pc.data=bytes(20)
            pc.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity','time'))]
            if cloud:pubs['cloud'].publish(pc)
            terrain=PointCloud2();terrain.header.frame_id='map';terrain.header.stamp=stamp;terrain.height=1
            points=[(a*.1,b*.1,0.,0.) for a in range(-5,31) for b in range(-10,11)]
            if blocked:
                points += [(.4,b*.03,.2,.5) for b in range(-60,61)]
            terrain.width=len(points);terrain.point_step=16;terrain.row_step=16*len(points)
            terrain.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity'))]
            terrain.data=b''.join(struct.pack('<ffff',*point) for point in points);pubs['terrain'].publish(terrain)
            rclpy.spin_once(node,timeout_sec=.02)
            del commands[:-500];del statuses[:-100]
    def service(name):
        client=node.create_client(Trigger,'/go2/mission/'+name+'_navigation')
        assert client.wait_for_service(timeout_sec=5.)
        future=client.call_async(Trigger.Request());until=time.monotonic()+5.
        while not future.done() and time.monotonic()<until:pump(.05)
        assert future.done() and future.result().success
        node.destroy_client(client)
    def goal():
        msg=PointStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg();msg.point.x=1.
        pubs['goal'].publish(msg)
    csv_path=root/'runtime'/('nav_cli_smoke_'+uuid.uuid4().hex+'.csv')
    recorder=subprocess.Popen(['ros2','run','go2_navigation_bridge','nav_status.py','--seconds','5','--csv',str(csv_path)],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(recorder)
    pump(3.)
    assert statuses and statuses[-1]['navigation_ready'],statuses[-1:]
    assert not statuses[-1]['command_enabled']
    service('resume');goal();pump(3.)
    assert any(v[0]>.05 for v in commands),statuses[-1:]
    commands.clear();pump(1.0,blocked=True)
    assert commands[-1]==(0.,0.,0.),'blocked local route continued nonzero velocity'
    pump(1.0)
    assert any(v[0]>.05 for v in commands[-20:]),statuses[-1:]
    pump(.8,cloud=False)
    assert commands[-1]==(0.,0.,0.) and not statuses[-1]['navigation_ready']
    pump(.8)
    assert statuses[-1]['command_enabled'],statuses[-1:]
    service('cancel');pump(.4)
    assert commands[-1]==(0.,0.,0.) and statuses[-1]['cancel_latched']
    service('resume');pump(.4)
    assert commands[-1]==(0.,0.,0.),'old local goal/path replayed after resume'
    cli=subprocess.Popen(['ros2','run','go2_navigation_bridge','nav_goal.py','--stage','local','--relative','--x','1.0','--y','0.0','--resume'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    processes.append(cli)
    until=time.monotonic()+8.
    while cli.poll() is None and time.monotonic()<until:pump(.05)
    assert cli.poll()==0,cli.communicate(timeout=1.)[0]
    pump(1.)
    assert statuses[-1]['command_enabled'],statuses[-1:]
    x=.9;pump(1.4)
    assert statuses[-1]['goal']['state']=='SUCCEEDED',statuses[-1:]
    assert commands[-1]==(0.,0.,0.)
    assert not node.get_publishers_info_by_topic('/lowcmd')
    assert not node.get_publishers_info_by_topic('/api/sport/request')
    assert recorder.poll()==0,'bounded CSV recorder did not finish'
    with csv_path.open() as file:rows=list(csv.DictReader(file))
    assert len(rows)>=10 and all(json.loads(row['status_json']) for row in rows),'NAV CSV missing snapshots'
    print('PASS operator relative goal CLI + bounded CSV recorder')
    print('PASS installed local+follower+bridge: forward, obstacle wall stop/recovery, cloud dropout/recovery, cancel/new goal, local arrival; no LowCmd/Sport')
finally:
    for process in processes:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
    for process in processes:
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    for log in logs:log.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
