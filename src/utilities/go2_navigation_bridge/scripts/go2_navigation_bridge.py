#!/usr/bin/env python3
"""Readiness/cancel/velocity adapter. Never imports Unitree SDK or arm SDK."""
import json
import math
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String, Bool, Int8
from std_srvs.srv import Trigger
from navigation_gate import NavigationGate
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

class NavigationBridge(Node):
    def __init__(self):
        super().__init__('go2_navigation_bridge')
        from rcl_interfaces.msg import ParameterDescriptor
        immutable = ParameterDescriptor(read_only=True)
        def param(name, value):
            return self.declare_parameter(name, value, immutable).value
        mode = param('input_mode', 'stack')
        self.local_route = param('local_route', False)
        if mode == 'command_test' and self.local_route:
            raise ValueError('local_route is only for stack mode')
        self.gate = NavigationGate(mode)
        self.session = uuid.uuid4().hex
        self.active_hint = False
        self.local_enabled = False
        self.last_odom_frame = ''
        self.active_owner_present = False
        self.last_graph_check = 0.
        self.last_tf_check = 0.
        self.tf_error = "not_observed"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.ready_pub = self.create_publisher(String, '/go2/mission/readiness', 1)
        self.status_pub = self.create_publisher(String, '/go2/navigation/status', 1)
        self.stop_pub = self.create_publisher(Int8, '/stop', 1)
        self.active_pub = self.create_publisher(Bool, '/navigation_active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if self.local_route else None
        self.create_subscription(TwistStamped, '/navigation/cmd_vel', self.command, 1)
        self.create_subscription(String, '/go2/locomotion_status', self.locomotion, 1)
        self.create_subscription(Odometry, '/state_estimation', self.odom, qos_profile_sensor_data)
        self.create_subscription(Imu, param('imu_topic', '/utlidar/transformed_raw_imu'), self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, param('lidar_topic', '/utlidar/transformed_cloud'), self.cloud, qos_profile_sensor_data)
        self.create_subscription(Path, '/path', self.path, 1)
        self.create_subscription(Int8, '/stop', self.stop, 1)
        if not self.local_route:
            self.create_subscription(Bool, '/navigation_active', self.active, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/go2/mission/cancel_navigation', self.cancel)
        self.create_service(Trigger, '/go2/mission/resume_navigation', self.resume)
        self.gate.observe('stop', True, time.monotonic())
        self.create_timer(.02, self.tick)
        self.get_logger().info('NAV request adapter mode=' + mode + '; no locomotion/Sport ownership')

    def header_age(self, msg):
        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000 or (stamp.sec == 0 and stamp.nanosec == 0):
            return float('inf')
        return (self.get_clock().now().nanoseconds - stamp.sec * 1000000000 - stamp.nanosec) / 1e9

    def header_valid(self, msg):
        age = self.header_age(msg)
        return bool(msg.header.frame_id) and math.isfinite(age) and 0 <= age <= .5

    def command(self, msg):
        self.gate.accept((msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z), self.header_age(msg), msg.header.frame_id, time.monotonic())

    def locomotion(self, msg):
        try:
            status = yaml.safe_load(msg.data)
            ready = status['state'] == 'ACTIVE' and status['phase'] in ('RL_ZERO', 'RL_ACTIVE') and status['output_enabled'] and not status['fault_latched'] and status['command_source'] == 'NAVIGATION'
        except (KeyError, TypeError, yaml.YAMLError):
            ready = False
        self.gate.locomotion(ready, time.monotonic())

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        valid = self.header_valid(msg) and msg.header.frame_id in ('map','camera_init') and msg.child_frame_id == 'aft_mapped' and all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and .5 < sum(x*x for x in (q.x,q.y,q.z,q.w)) < 1.5
        self.last_odom_frame = msg.header.frame_id
        self.gate.observe('odom', valid, time.monotonic(), self.header_age(msg))

    def imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.gate.observe('imu', self.header_valid(msg) and all(math.isfinite(x) for x in (a.x,a.y,a.z,g.x,g.y,g.z)), time.monotonic(), self.header_age(msg))

    def cloud(self, msg):
        names = {f.name for f in msg.fields}
        valid = self.header_valid(msg) and msg.width * msg.height > 0 and {'x','y','z','intensity'}.issubset(names) and bool(names & {'time','t'}) and len(msg.data) >= msg.row_step * msg.height and msg.point_step > 0
        self.gate.observe('lidar', valid, time.monotonic(), self.header_age(msg))

    def path(self, msg):
        valid = self.header_valid(msg) and len(msg.poses) > 1 and all(math.isfinite(v) for p in msg.poses for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z))
        self.gate.observe('path', valid, time.monotonic(), self.header_age(msg))

    def active(self, msg):
        self.active_hint = bool(msg.data)
        self.gate.planner_active(self.active_hint, time.monotonic())

    def stop(self, msg):
        self.gate.observe('stop', msg.data == 0, time.monotonic())

    def cancel(self, request, response):
        self.gate.cancel()
        self.local_enabled = False
        self.stop_pub.publish(Int8(data=2))
        self.tick()
        response.success = True
        response.message = 'velocity revoked; cancel latched; resume requires a new request/path'
        return response

    def resume(self, request, response):
        self.gate.resume()
        self.gate.observe('path', False, time.monotonic())
        self.local_enabled = True
        self.stop_pub.publish(Int8(data=0))
        self.gate.observe('stop', True, time.monotonic())
        response.success = True
        response.message = 'cancel cleared; waiting for new command and, in stack mode, fresh path'
        return response

    def tick(self):
        now = time.monotonic()
        if self.gate.mode == 'stack' and now - self.last_tf_check >= .2:
            self.last_tf_check = now
            try:
                lookup = self.tf_buffer.lookup_transform('map','vehicle',Time())
                transform = lookup.transform
                stamp_ns = lookup.header.stamp.sec * 1000000000 + lookup.header.stamp.nanosec
                tf_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                time_valid = stamp_ns == 0 or 0 <= tf_age <= .5
                self.tf_error = "" if time_valid else "tf_timestamp_stale"
                v, q = transform.translation, transform.rotation
                valid = time_valid and all(math.isfinite(x) for x in (v.x,v.y,v.z,q.x,q.y,q.z,q.w)) and .5 < sum(x*x for x in (q.x,q.y,q.z,q.w)) < 1.5
            except Exception as error:
                self.tf_error = str(error)
                valid = False
            self.gate.observe('tf',valid,now)
        # FAR owns navigation_active in FAR mode and publishes on change; cache
        # its intent, but require separate fresh odom/path/command each tick.
        if self.local_route:
            self.active_hint = self.local_enabled and not self.gate.cancelled and self.gate.localization_ready(now)
            self.active_pub.publish(Bool(data=self.active_hint))
        if self.gate.mode == 'stack' and not self.local_route:
            # FAR publishes intent on change, so its publisher must remain alive.
            if now - self.last_graph_check >= .2:
                self.active_owner_present = self.count_publishers('/navigation_active') == 1
                self.last_graph_check = now
            self.gate.planner_active(self.active_hint and self.active_owner_present, now)
        elif self.local_route:
            self.gate.planner_active(self.active_hint, now)
        value = self.gate.output(now)
        enabled = self.gate.enabled(now)
        ready = String(data=json.dumps(dict(observed_ns=time.monotonic_ns(), session_id=self.session, navigation_ready=self.gate.localization_ready(now), perception_ready=False, command_enabled=enabled)))
        self.ready_pub.publish(ready)
        msg = TwistStamped()
        msg.header.frame_id = 'vehicle'
        # Preserve request age rather than re-stamping an old nonzero command.
        age = (now - self.gate.request_received + self.gate.request_header_age) if value != (0.,0.,0.) else 0.
        msg.header.stamp = Time(nanoseconds=max(0, self.get_clock().now().nanoseconds - int(age * 1e9))).to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = value
        self.cmd_pub.publish(msg)
        self.status_pub.publish(String(data=json.dumps(dict(session_id=self.session, input_mode=self.gate.mode, navigation_ready=self.gate.localization_ready(now), command_enabled=enabled, cancel_latched=self.gate.cancelled, block_reason=self.gate.block_reason, tf_error=self.tf_error, health_ages={name: None if stamp is None else now-stamp for name,stamp in self.gate.health.items()}, odom_frame=self.last_odom_frame, command=list(value)))))

def main():
    rclpy.init()
    node = NavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
