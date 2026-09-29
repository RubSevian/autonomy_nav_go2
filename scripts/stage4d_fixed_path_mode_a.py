#!/usr/bin/env python3
"""Mode A: publish one fixed vehicle-frame path only when localPlanner is absent."""
import argparse,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visibility_graph_msg.msg import LocalPathConstraint
QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def main():
 p=argparse.ArgumentParser();p.add_argument('--length',type=float,default=2.0);p.add_argument('--step',type=float,default=.05);p.add_argument('--duration',type=float,default=30.0);a=p.parse_args()
 rclpy.init();n=Node('stage4d_fixed_path_mode_a');pp=n.create_publisher(Path,'/path',QOS);cp=n.create_publisher(LocalPathConstraint,'/local_path_constraint',QOS)
 end=time.monotonic()+5
 while pp.get_subscription_count()==0 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 # A second publisher means Mode A would no longer isolate follower/RL.
 if n.count_publishers('/path') != 1: raise RuntimeError('Mode A requires localPlanner /path publisher disabled; refusing mixed path sources')
 path=Path();path.header.frame_id='vehicle';path.header.stamp=n.get_clock().now().to_msg()
 for i in range(int(a.length/a.step)+1):
  pose=PoseStamped();pose.pose.position.x=i*a.step;pose.pose.orientation.w=1.;path.poses.append(pose)
 c=LocalPathConstraint();c.header=path.header;c.path_revision=1;c.candidate_group_id=0;c.candidate_rotation_id=0;c.candidate_direction=1;c.switch_reason='MODE_A_FIXED_STRAIGHT';c.reason='MODE_A_FIXED_STRAIGHT';c.speed_scale=1.
 until=time.monotonic()+a.duration
 try:
  while rclpy.ok() and time.monotonic()<until:
   path.header.stamp=n.get_clock().now().to_msg();c.header=path.header;cp.publish(c);pp.publish(path);rclpy.spin_once(n,timeout_sec=.05);time.sleep(.05)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
