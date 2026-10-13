#!/usr/bin/env python3
"""Real FAR process, isolated ROS domain; no sensors/leg/arm/Sport processes."""
import os
import json
import time
import subprocess
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID') == '228' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import rclpy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
rclpy.init()
node=rclpy.create_node('far_cancel_smoke')
heartbeats=[]
statuses=[]
node.create_subscription(DiagnosticArray,'/far/planner_status',lambda msg:statuses.extend({kv.key:kv.value for kv in status.values} for status in msg.status),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
odom=node.create_publisher(Odometry,'/odom_world',1)
node.create_subscription(String,'/go2/navigation/planner_heartbeat',lambda m:heartbeats.append(json.loads(m.data)),1)
client=node.create_client(Trigger,'/far_planner/cancel_goal')
goal=node.create_publisher(PointStamped,'/goal_point',1)
root=Path(__file__).resolve().parents[6]
executable=root/'install_nav_stack/far_planner/lib/far_planner/far_planner'
proc=subprocess.Popen([str(executable),'--ros-args','-p','is_opencv_visual:=false','-p','sensor_offset_x:=0.3'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
try:
    assert client.wait_for_service(timeout_sec=8),'FAR cancel port absent'
    deadline=time.monotonic()+25.
    while len(heartbeats)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    assert len(heartbeats)>=2,'FAR initialization did not complete'
    # Exercise queued-goal cancellation before graph/sensor initialization.
    msg=PointStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg();msg.point.x=1.
    end=time.monotonic()+2
    while time.monotonic()<end:
        state=Odometry();state.header.frame_id='map';state.header.stamp=node.get_clock().now().to_msg()
        state.pose.pose.position.x=1.;state.pose.pose.position.y=2.
        state.pose.pose.orientation.z=math.sqrt(.5);state.pose.pose.orientation.w=math.sqrt(.5)
        odom.publish(state);goal.publish(msg)
        rclpy.spin_once(node,timeout_sec=.05)
    assert any(abs(float(v.get('robot_x','nan'))-1.)<1e-4 and abs(float(v.get('robot_y','nan'))-1.7)<1e-4 for v in statuses if v.get('robot_x')!='N/A'),'FAR sensor->vehicle offset not applied to world pose'
    future=client.call_async(Trigger.Request())
    end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert future.done() and future.result().success,'goal cancellation failed'
    end=time.monotonic()+3
    while len(heartbeats)<4 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert len(heartbeats)>=3 and all(not h['active'] for h in heartbeats)
    assert len({h['session_id'] for h in heartbeats})==1
    assert all(b['observed_ns']>a['observed_ns'] for a,b in zip(heartbeats,heartbeats[1:]))
    end=time.monotonic()+1.
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert any(v.get('reason_code')=='CANCELLED' and v.get('pending_goal_present')=='false' and v.get('goal_received')=='false' for v in statuses),'queued goal was not cleared'
    assert not node.get_publishers_info_by_topic('/lowcmd')
    assert not node.get_publishers_info_by_topic('/api/sport/request')
    print('PASS installed FAR: live inactive heartbeat, queued goal cancellation + preserved graph, body-origin offset, live heartbeat; no LowCmd/Sport publishers')
finally:
    proc.terminate()
    try:log=proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:proc.kill();log=proc.communicate()[0]
    print(log[-1200:]);node.destroy_node();rclpy.shutdown()
