#!/usr/bin/env python3
"""Operator-issued local/FAR goal. Never enables R3, arm, or Sport."""
import argparse
import math
import time
import rclpy
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('local', 'far'), required=True)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, default=0.)
    parser.add_argument('--relative', action='store_true', help='Interpret XYZ in the current vehicle frame using fresh real TF')
    parser.add_argument('--resume', action='store_true', help='Clear NAV cancel before issuing a NEW goal')
    args, ros_args = parser.parse_known_args()
    if not all(math.isfinite(x) for x in (args.x,args.y,args.z)):
        parser.error('Coordinates must be finite map coordinates')
    rclpy.init(args=ros_args)
    node = rclpy.create_node('nav_goal_request')
    try:
        if args.resume:
            client = node.create_client(Trigger, '/go2/mission/resume_navigation')
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('NAV bridge resume service not available')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError('NAV resume was not acknowledged; goal not sent')
        topic = '/goal_point' if args.stage == 'far' else '/way_point'
        pub = node.create_publisher(PointStamped, topic, 1)
        until = time.monotonic()+5.
        while rclpy.ok() and pub.get_subscription_count() < 2 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if pub.get_subscription_count() < 2:
            raise RuntimeError('Expected both bridge and planner goal subscribers; check stage and launch')
        target = (args.x,args.y,args.z)
        if args.relative:
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            until = time.monotonic()+5.
            transform = None
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
                try:
                    candidate = buffer.lookup_transform('map','vehicle',Time())
                    stamp = candidate.header.stamp.sec*1000000000+candidate.header.stamp.nanosec
                    age = (node.get_clock().now().nanoseconds-stamp)/1e9
                    if stamp > 0 and 0 <= age <= .5:
                        transform = candidate.transform
                        break
                except Exception:
                    continue
            if transform is None:
                raise RuntimeError('Fresh real map->vehicle TF unavailable; no goal sent')
            q,t = transform.rotation,transform.translation
            norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if not math.isfinite(norm) or not .5 < norm < 1.5:
                raise RuntimeError('Invalid map->vehicle quaternion; no goal sent')
            # Rotate vector by normalized quaternion; relative XYZ includes height.
            scale = math.sqrt(norm)
            x,y,z,w = q.x/scale,q.y/scale,q.z/scale,q.w/scale
            vx,vy,vz = target
            tx,ty,tz = 2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
            target = (t.x+vx+w*tx+y*tz-z*ty, t.y+vy+w*ty+z*tx-x*tz, t.z+vz+w*tz+x*ty-y*tx)
        if not all(math.isfinite(value) for value in target):
            raise RuntimeError('Transformed goal is not finite; no goal sent')
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.point.x,msg.point.y,msg.point.z = target
        # Same stamped request retransmitted briefly for DDS discovery; no new goals.
        for _ in range(3):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
        print(f'Sent map goal {target} to {topic}; inspect nav_status for acceptance/result')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
