#!/usr/bin/env python3
"""Actual localPlanner/follower, ideal planar plant; isolated ROS, no robot APIs.

Exercises waypoint approach with the installed binaries. FAR completion is NOT
simulated: the run stops at first entry into the unchanged XY arrival tolerance.
This is a navigation regression, not a locomotion/SLAM accuracy measurement.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Requires localhost ROS_DOMAIN_ID=228')

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_profile import load_profile, component_parameters


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--goal-x',type=float,default=.506567)
    parser.add_argument('--goal-y',type=float,default=-.502358)
    parser.add_argument('--legacy',action='store_true',help='Reproduce former coupling to arrival tolerance')
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--expect',choices=['arrive','miss'],default='arrive')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
    params=component_parameters(profile)
    if args.legacy:
        for name in ('localPlanner','pathFollower'):
            params[name]['goalCloseDis']=profile['local']['goal_tolerance_m']
    params['localPlanner']['pathFolder']=str(Path(get_package_share_directory('local_planner'))/'paths')
    processes=[];logs=[];rclpy.init();node=rclpy.create_node('near_goal_regression')
    activeq=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub=node.create_publisher(Odometry,'/state_estimation',10)
    from sensor_msgs.msg import PointCloud2
    cloud_pub=node.create_publisher(PointCloud2,'/terrain_map',10)
    goal_pub=node.create_publisher(PointStamped,'/way_point',10)
    active_pub=node.create_publisher(Bool,'/navigation_active',activeq)
    command=[0.,0.,0.];command_time=0.;diagnostics={};trace=[]
    def on_command(msg):
        nonlocal command_time
        command[:]=[msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z]
        command_time=time.monotonic()
    def on_status(msg):
        for s in msg.status:diagnostics[s.name]={v.key:v.value for v in s.values}
    node.create_subscription(TwistStamped,'/navigation/cmd_vel',on_command,10)
    node.create_subscription(DiagnosticArray,'/local_planner/status',on_status,activeq)
    node.create_subscription(DiagnosticArray,'/path_follower/status',on_status,activeq)
    x=y=yaw=0.;arrived=False;first_path=None
    try:
        for name in ('localPlanner','pathFollower'):
            executable=Path(get_package_prefix('local_planner'))/'lib/local_planner'/name
            argv=[str(executable),'--ros-args','-r','/cmd_vel:=/navigation/cmd_vel']
            for k,v in params[name].items():
                argv+=['-p',k+':='+(str(v).lower() if type(v) is bool else str(v))]
            log=open(args.output/(name+'.log'),'w');logs.append(log)
            processes.append(subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        started=last=time.monotonic();last_cloud=last_goal=0.;settling_start=None;halted=False
        while time.monotonic()-started<args.duration+3.:
            now=time.monotonic();dt=min(now-last,.05);last=now
            assert all(p.poll() is None for p in processes),'planner process exited'
            ready=now-started>=3.
            elapsed=now-started-3.
            applied=[max(-limit,min(limit,v)) for v,limit in zip(command,(profile['bridge']['vx_limit'],profile['bridge']['vy_limit'],profile['bridge']['wz_limit']))]
            if not ready or arrived or now-command_time>.25:applied=[0.,0.,0.]
            vx,vy,wz=applied
            x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*dt
            y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*dt
            yaw+=wz*dt
            distance=math.hypot(args.goal_x-x,args.goal_y-y)
            if ready and distance<=profile['local']['goal_tolerance_m'] and not arrived:
                arrived=True;settling_start=now
            stamp=node.get_clock().now().to_msg()
            od=Odometry();od.header.frame_id='map';od.child_frame_id='aft_mapped';od.header.stamp=stamp
            sx=profile['robot']['sensor_offset_x_m'];sy=profile['robot']['sensor_offset_y_m']
            od.pose.pose.position.x=x+math.cos(yaw)*sx-math.sin(yaw)*sy
            od.pose.pose.position.y=y+math.sin(yaw)*sx+math.cos(yaw)*sy
            od.pose.pose.orientation.z=math.sin(yaw/2);od.pose.pose.orientation.w=math.cos(yaw/2)
            odom_pub.publish(od)
            if now-last_cloud>=.05:
                fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
                floor=[(x+i*.1,y+j*.1,0.,0.) for i in range(-15,16) for j in range(-15,16)]
                cloud_pub.publish(point_cloud2.create_cloud(od.header,fields,floor));last_cloud=now
            if now-last_goal>=.2:
                active_pub.publish(Bool(data=ready and not arrived))
                if ready and not arrived:
                    g=PointStamped();g.header=od.header;g.point.x=args.goal_x;g.point.y=args.goal_y;goal_pub.publish(g)
                last_goal=now
            rclpy.spin_once(node,timeout_sec=.01)
            if ready:
                trace.append(dict(t=elapsed,x=x,y=y,yaw=yaw,distance=distance,raw_command=list(command),applied=applied,candidate=diagnostics.get('local_planner',{}).get('candidate_id')))
                if first_path is None and diagnostics.get('local_planner',{}).get('path_found')=='true':first_path=diagnostics['local_planner'].copy()
            if arrived and now-settling_start>1.:
                halted=all(abs(v)<1e-8 for v in command);break
        assert not node.get_publishers_info_by_topic('/lowcmd')
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        result=dict(goal=[args.goal_x,args.goal_y],legacy=args.legacy,near_goal_distance_m=params['localPlanner']['goalCloseDis'],tolerance_m=profile['local']['goal_tolerance_m'],arrived=arrived,stopped_on_inactive=halted,min_distance_m=min((r['distance'] for r in trace),default=None),first_entry_s=next((r['t'] for r in trace if r['distance']<=profile['local']['goal_tolerance_m']),None),first_path=first_path,trace=trace,limitations='Ideal planar kinematics, empty floor, bridge clamps only. No FAR, RL, camera, SLAM, or physical robot.')
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('trace','first_path')},indent=2),flush=True)
        assert trace and first_path,'No valid planning observed'
        assert arrived==(args.expect=='arrive'),'Unexpected approach result'
        if arrived:assert halted,'Follower did not stop on navigation inactive'
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
