#!/usr/bin/env python3
"""Operator-only bounded NAV request smoke; does not arm legs or change modes."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

def main():
    p=argparse.ArgumentParser()
    for name in ('vx','vy','wz'):p.add_argument('--'+name,type=float,default=0.)
    p.add_argument('--seconds',type=float,default=1.)
    args,ros_args=p.parse_known_args()
    value=(args.vx,args.vy,args.wz)
    if not all(math.isfinite(x) and abs(x)<=limit for x,limit in zip(value,(.2,.1,.1))):raise ValueError('NAV limits vx±.2, vy/wz±.1')
    if not math.isfinite(args.seconds) or not 0<args.seconds<=10:raise ValueError('seconds must be (0,10]')
    rclpy.init(args=ros_args);n=Node('nav_manual_request');pub=n.create_publisher(TwistStamped,'/navigation/cmd_vel',1)
    def send(v):
        msg=TwistStamped();msg.header.frame_id='vehicle';msg.header.stamp=n.get_clock().now().to_msg()
        msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z=v;pub.publish(msg)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.seconds:
            send(value);rclpy.spin_once(n,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            for _ in range(3):send((0.,0.,0.));rclpy.spin_once(n,timeout_sec=.02)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
