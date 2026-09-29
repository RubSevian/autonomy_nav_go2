#!/usr/bin/env python3
"""ROS-native evidence recorder for Stage4D navigation isolation modes."""
import argparse, json, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Bool, Int64
from visibility_graph_msg.msg import LocalPathConstraint
TRANSIENT=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
class Probe(Node):
 def __init__(self,out):
  super().__init__('stage4d_navigation_probe'); self.out=Path(out); self.events=[]; self.last={}; self.pose_time={}
  self.create_subscription(DiagnosticArray,'/far/planner_status',self.diag('far'),TRANSIENT)
  self.create_subscription(Bool,'/navigation_active',self.boolean('navigation_active'),TRANSIENT)
  self.create_subscription(PointStamped,'/way_point',self.waypoint,10)
  self.create_subscription(DiagnosticArray,'/local_planner/status',self.diag('local_planner'),TRANSIENT)
  self.create_subscription(NavPath,'/path',self.path,10)
  self.create_subscription(LocalPathConstraint,'/local_path_constraint',self.constraint,TRANSIENT)
  self.create_subscription(DiagnosticArray,'/path_follower/status',self.diag('path_follower'),TRANSIENT)
  self.create_subscription(TwistStamped,'/cmd_vel',self.cmd,10)
  self.create_subscription(Odometry,'/state_estimation',self.odom,10)
  self.create_subscription(Odometry,'/sim/ground_truth_odom',lambda m:self.odom(m,'ground_truth_pose'),10)
  self.create_subscription(Int64,'/mujoco/non_floor_contact_count',lambda m:self.add('non_floor_contacts',int(m.data)),TRANSIENT)
  self.create_subscription(Int64,'/mujoco/unexpected_floor_contact_count',lambda m:self.add('unexpected_floor_contacts',int(m.data)),TRANSIENT)
  self.create_subscription(Bool,'/far_reach_goal_status',self.boolean('goal_reached'),10)
 def add(self,k,v):
  key=json.dumps(v,sort_keys=True,default=str)
  if self.last.get(k)==key:return
  self.last[k]=key; self.events.append({'wall_time':time.time(),'event':k,'value':v})
 def boolean(self,k):return lambda m:self.add(k,bool(m.data))
 def diag(self,k):
  def cb(m):
   for s in m.status:
    v={i.key:i.value for i in s.values}; self.add(k,{'state':v.get('state',s.message),'reason':v.get('reason_code','N/A'),'path_found':v.get('path_found','N/A'),'path_size':v.get('published_path_size',v.get('path_size','N/A')),'candidate':v.get('candidate_id',v.get('selected_group_id','N/A')),'rotation':v.get('selected_rotation_id','N/A'),'direction':v.get('selected_direction','N/A'),'switches':v.get('path_switch_count','N/A'),'refreshes':v.get('path_refresh_count','N/A'),'switch_reason':v.get('switch_reason','N/A'),'motion_model':v.get('motion_model','N/A'),'tracking_index':v.get('tracking_point_index','N/A'),'current_score':v.get('current_score','N/A'),'best_score':v.get('best_score','N/A'),'yaw_error':v.get('yaw_error','N/A'),'heading_error':v.get('path_heading_error','N/A'),'constraint_matches_path':v.get('constraint_matches_path','N/A')})
  return cb
 def waypoint(self,m):self.add('waypoint',{'frame':m.header.frame_id,'x':m.point.x,'y':m.point.y,'z':m.point.z})
 def path(self,m):self.add('path',{'frame':m.header.frame_id,'size':len(m.poses)})
 def constraint(self,m):self.add('constraint',{'revision':m.path_revision,'group':m.candidate_group_id,'rotation':m.candidate_rotation_id,'direction':m.candidate_direction,'narrow':m.candidate_narrow_mode})
 def cmd(self,m):
  v=m.twist
  if abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)>1e-5:self.add('cmd_vel',{'vx':v.linear.x,'vy':v.linear.y,'wz':v.angular.z})
 def odom(self,m,key='robot_pose'):
  now=time.monotonic()
  if now-self.pose_time.get(key,0)<0.1:return
  self.pose_time[key]=now
  q=m.pose.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); self.add(key,{'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':yaw})
 def write(self):self.out.parent.mkdir(parents=True,exist_ok=True);self.out.write_text(json.dumps({'events':self.events},indent=2)+'\n')
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=60.0);a=p.parse_args();rclpy.init();n=Probe(a.output);end=time.monotonic()+a.duration
 try:
  while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=0.2)
 finally:n.write();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
