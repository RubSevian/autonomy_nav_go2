#!/usr/bin/env python3
"""Readable NAV diagnostics and optional bounded CSV; observational only."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--once', action='store_true')
    p.add_argument('--csv', type=Path, help='Optional NEW CSV path; recording is off by default')
    args, ros_args = p.parse_known_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        p.error('seconds must be (0,3600]')
    rclpy.init(args=ros_args)
    node = rclpy.create_node('nav_status')
    seen = []  # one-element bounded mailbox
    def receive(msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data,dict):
                seen[:] = [data]
        except (TypeError,ValueError):
            pass
    node.create_subscription(String, '/go2/navigation/status', receive, 1)
    output = None
    try:
        writer = None
        if args.csv:
            args.csv.expanduser().parent.mkdir(parents=True, exist_ok=True)
            output = args.csv.expanduser().open('x', newline='')
            writer = csv.writer(output)
            writer.writerow(('monotonic_s','session_id','goal_revision','goal_state','elapsed_s','error_xy_m',
                             'navigation_ready','command_enabled','block_reason','vx','vy','wz','status_json'))
        start,last_print,last_seen = time.monotonic(),0.,None
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if seen and seen[0] is not last_seen:
                status = seen[0]
                last_seen = status
                now = time.monotonic()
                goal = status.get('goal',{})
                if writer:
                    writer.writerow((now,status.get('session_id'),goal.get('revision'),goal.get('state'),
                                     goal.get('elapsed_s'),goal.get('error_xy_m'),status.get('navigation_ready'),
                                     status.get('command_enabled'),status.get('block_reason'),*status.get('command',[0.,0.,0.]),json.dumps(status)))
                if args.once or now-last_print >= 1.:
                    print(json.dumps(status,ensure_ascii=False,indent=2),flush=True)
                    last_print = now
                if args.once:
                    break
        if not seen:
            raise RuntimeError('No NAV status observed; check environment and bridge launch')
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
