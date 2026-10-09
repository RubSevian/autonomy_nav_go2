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
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
rclpy.init()
node=rclpy.create_node('far_cancel_smoke')
heartbeats=[]
node.create_subscription(String,'/go2/navigation/planner_heartbeat',lambda m:heartbeats.append(json.loads(m.data)),1)
client=node.create_client(Trigger,'/far_planner/cancel_goal')
goal=node.create_publisher(PointStamped,'/goal_point',1)
root=Path(__file__).resolve().parents[6]
executable=root/'install_nav_stack/far_planner/lib/far_planner/far_planner'
proc=subprocess.Popen([str(executable),'--ros-args','-p','is_opencv_visual:=false'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
try:
    assert client.wait_for_service(timeout_sec=8),'FAR cancel port absent'
    # Exercise queued-goal cancellation before graph/sensor initialization.
    msg=PointStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg();msg.point.x=1.
    goal.publish(msg)
    end=time.monotonic()+1
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    future=client.call_async(Trigger.Request())
    end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert future.done() and future.result().success,'goal cancellation failed'
    end=time.monotonic()+3
    while len(heartbeats)<4 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert len(heartbeats)>=3 and all(not h['active'] for h in heartbeats)
    assert len({h['session_id'] for h in heartbeats})==1
    assert all(b['observed_ns']>a['observed_ns'] for a,b in zip(heartbeats,heartbeats[1:]))
    assert not node.get_publishers_info_by_topic('/lowcmd')
    assert not node.get_publishers_info_by_topic('/api/sport/request')
    print('PASS installed FAR: live inactive heartbeat, queued goal cancel acknowledgement; no LowCmd/Sport publishers')
finally:
    proc.terminate()
    try:log=proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:proc.kill();log=proc.communicate()[0]
    print(log[-1200:]);node.destroy_node();rclpy.shutdown()
