#!/usr/bin/env python3
"""Isolated ROS command/service smoke; synthetic FSM, no robot/SDK/LowCmd node."""
import os
import json
import sys
import time
import subprocess
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Run only in localhost ROS_DOMAIN_ID=228')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, TransformStamped, PointStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Bool, Int8
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
rclpy.init()
n = Node('nav_command_smoke')
out = []
ready = []
diagnostics = []
n.create_subscription(String,'/go2/navigation/status',lambda m: diagnostics.append(json.loads(m.data)),1)
n.create_subscription(TwistStamped,'/cmd_vel',lambda m: out.append((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),1)
n.create_subscription(String,'/go2/mission/readiness',lambda m: ready.append(json.loads(m.data)),1)
fsm=n.create_publisher(String,'/go2/locomotion_status',1)
requests=n.create_publisher(TwistStamped,'/navigation/cmd_vel',1)
stack=os.environ.get('NAV_SMOKE_STACK')=='1'
stop_pub=n.create_publisher(Int8,'/stop',1)
goal_pub=n.create_publisher(PointStamped,'/goal_point',1)
cancel_calls=[]
def cancel_goal(req,res):
    cancel_calls.append(True);res.success=True;return res
cancel_srv=n.create_service(Trigger,'/far_planner/cancel_goal',cancel_goal)
broadcaster=StaticTransformBroadcaster(n)
tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='camera_init';tf.transform.rotation.w=1.
base=TransformStamped();base.header.frame_id='aft_mapped';base.child_frame_id='vehicle';base.transform.rotation.w=1.
broadcaster.sendTransform([tf,base])
dynamic_broadcaster=TransformBroadcaster(n)
cloud_pub=n.create_publisher(PointCloud2,'/utlidar/transformed_cloud',1)
imu_pub=n.create_publisher(Imu,'/utlidar/transformed_raw_imu',1)
odom_pub=n.create_publisher(Odometry,'/state_estimation',1)
path_pub=n.create_publisher(NavPath,'/path',1)
active_pub=n.create_publisher(Bool,'/navigation_active',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
heartbeat_pub=n.create_publisher(String,'/go2/navigation/planner_heartbeat',1)
bridge=Path(__file__).resolve().parents[1]/'scripts/go2_navigation_bridge.py'
proc=subprocess.Popen([sys.executable,str(bridge),'--ros-args','-p','input_mode:='+('stack' if stack else 'command_test'),'-p','far_route:='+str(stack).lower()],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
def pump(seconds, value=None, active=True, cloud=True, planner=True, goal=True, tf_stream=True):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if stack:
            stamp=n.get_clock().now().to_msg()
            if cloud:
                msg=PointCloud2();msg.header.frame_id='body';msg.header.stamp=stamp;msg.width=1;msg.height=1;msg.point_step=20;msg.row_step=20;msg.data=bytes(20)
                msg.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity','time'))]
                cloud_pub.publish(msg)
            msg=Imu();msg.header.frame_id='body';msg.header.stamp=stamp;msg.linear_acceleration.z=9.81;imu_pub.publish(msg)
            msg=Odometry();msg.header.frame_id='camera_init';msg.header.stamp=stamp;msg.child_frame_id='aft_mapped';msg.pose.pose.orientation.w=1.;odom_pub.publish(msg)
            msg=NavPath();msg.header.frame_id='vehicle';msg.header.stamp=stamp;msg.poses=[PoseStamped(),PoseStamped()];msg.poses[1].pose.position.x=.5;path_pub.publish(msg)
            if planner is not None:
                active_pub.publish(Bool(data=planner))
                heartbeat_pub.publish(String(data=json.dumps(dict(session_id='synthetic',observed_ns=time.monotonic_ns(),active=planner))))
            tf=TransformStamped();tf.header.frame_id='camera_init';tf.child_frame_id='aft_mapped';tf.header.stamp=stamp;tf.transform.rotation.w=1.
            if tf_stream:dynamic_broadcaster.sendTransform(tf)
            if goal:
                msg=PointStamped();msg.header.frame_id='map';msg.header.stamp=stamp;msg.point.x=1.;goal_pub.publish(msg)
        if active:
            fsm.publish(String(data=json.dumps(dict(state='ACTIVE',phase='RL_ZERO',output_enabled=True,fault_latched=False,command_source='NAVIGATION'))))
        if value is not None:
            msg=TwistStamped();msg.header.frame_id='vehicle';msg.header.stamp=n.get_clock().now().to_msg()
            msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z=value
            requests.publish(msg)
        rclpy.spin_once(n,timeout_sec=.02)
def service(name):
    client=n.create_client(Trigger,name)
    assert client.wait_for_service(timeout_sec=3),name
    f=client.call_async(Trigger.Request())
    until=time.monotonic()+3
    while not f.done() and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
    assert f.done() and f.result().success,name
try:
    startup_deadline=time.monotonic()+5
    while (not ready or not ready[-1]['navigation_ready'] or not out) and time.monotonic()<startup_deadline:
        pump(.2,active=False)
    assert ready and ready[-1]['navigation_ready'] and not ready[-1]['command_enabled'], (ready[-1:] , diagnostics[-1:])
    assert out and out[-1]==(0.,0.,0.)
    for value,expected in [((.12,0.,0.),(.12,0.,0.)),((-.12,0.,0.),(-.12,0.,0.)),((0.,.2,0.),(0.,.1,0.)),((0.,0.,-.2),(0.,0.,-.1))]:
        pump(.4,value)
        assert out[-1]==expected,(value,out[-1])
    pump(.4)
    assert out[-1]==(0.,0.,0.),'command stale must zero'
    pump(.2,(.1,0.,0.))
    service('/go2/mission/cancel_navigation')
    pump(.3,(.1,0.,0.))
    assert out[-1]==(0.,0.,0.) and not ready[-1]['command_enabled']
    service('/go2/mission/resume_navigation')
    pump(.15,goal=False)
    assert out[-1]==(0.,0.,0.),'resume cannot replay'
    pump(.3,(.1,0.,0.))
    assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
    stop_pub.publish(Int8(data=1));pump(.15,(.1,0.,0.))
    assert out[-1]==(0.,0.,0.)
    service('/go2/mission/resume_navigation');pump(.3,(.1,0.,0.))
    assert out[-1]==(0.,0.,0.),'resume must retain external stop'
    stop_pub.publish(Int8(data=0));pump(.3,(.1,0.,0.))
    assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
    assert len(n.get_publishers_info_by_topic('/stop'))==1,'bridge must not own external stop'
    assert ready[-1]['session_id'] and ready[-1]['command']==list(out[-1])
    if stack:
        assert cancel_calls,'real cancel service must reach FAR port'
        pump(.8,(.1,0.,0.),planner=None)
        assert out[-1]==(0.,0.,0.),'existing DDS publisher without heartbeat must not enable'
        pump(.3,(.1,0.,0.))
        assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
        pump(.8,(.1,0.,0.),tf_stream=False)
        assert out[-1]==(0.,0.,0.) and not ready[-1]['navigation_ready'],'stale/mismatched TF must revoke'
        pump(.3,(.1,0.,0.))
        assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
        n.destroy_publisher(heartbeat_pub)
        heartbeat_pub=n.create_publisher(String,'/go2/navigation/planner_heartbeat',1)
        pump(.8,(.1,0.,0.),planner=None)
        assert out[-1]==(0.,0.,0.),'replacement publisher cannot replay cached active'
        pump(.4,(.1,0.,0.))
        assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
        pump(.8,(.1,0.,0.),cloud=False)
        assert out[-1]==(0.,0.,0.) and not ready[-1]['navigation_ready'], 'LiDAR dropout must revoke velocity without leg output'
        pump(.3)
        assert out[-1]==(0.,0.,0.),'sensor recovery cannot replay old request'
        pump(.3,(.1,0.,0.))
        assert out[-1]==(.1,0.,0.),(out[-1:],ready[-1:],diagnostics[-1:])
        pump(.3,(.1,0.,0.),planner=False)
        assert out[-1]==(0.,0.,0.),'planner stop must zero'
    assert not n.get_publishers_info_by_topic('/lowcmd'), 'unexpected LowCmd publisher in isolated domain'
    assert not n.get_publishers_info_by_topic('/api/sport/request'), 'unexpected Sport publisher'
    print(('STACK ' if stack else 'COMMAND_TEST ')+'PASS ROS synthetic command directions/clamp/timeout/cancel/resume; zero LowCmd/Sport publishers')
finally:
    proc.terminate()
    try:log=proc.communicate(timeout=4)[0]
    except subprocess.TimeoutExpired:
        proc.kill();log=proc.communicate()[0]
    print(log[-1500:])
    n.destroy_node();rclpy.shutdown()
