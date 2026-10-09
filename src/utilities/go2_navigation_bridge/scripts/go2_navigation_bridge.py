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
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String, Bool, Int8
from std_srvs.srv import Trigger
from navigation_gate import NavigationGate
from navigation_contract import valid_cloud_layout, OdometryHistory
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
        self.far_route = param('far_route', False)
        if self.far_route and (mode != 'stack' or self.local_route):
            raise ValueError('far_route requires stack without local_route')
        self.gate = NavigationGate(mode)
        self.goal_after_ns = self.get_clock().now().nanoseconds
        self.far_cancel_pending = False
        self.far_cancel_future = None
        self.far_cancel_started = 0.
        self.far_cancel_retry_at = 0.
        self.far_cancel_client = self.create_client(Trigger, '/far_planner/cancel_goal') if self.far_route else None
        self.odom_history = OdometryHistory()
        self.session = uuid.uuid4().hex
        self.active_hint = False
        self.local_enabled = False
        self.last_odom_frame = ''
        self.active_owner_gid = None
        self.active_owner_since_ns = 0
        self.active_session = ""
        self.active_observed_ns = 0
        self.last_graph_check = 0.
        self.last_tf_check = 0.
        self.tf_error = "not_observed"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.ready_pub = self.create_publisher(String, '/go2/mission/readiness', 1)
        self.status_pub = self.create_publisher(String, '/go2/navigation/status', 1)
        self.active_pub = self.create_publisher(Bool, '/navigation_active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if self.local_route else None
        self.create_subscription(TwistStamped, '/navigation/cmd_vel', self.command, 1)
        self.create_subscription(String, '/go2/locomotion_status', self.locomotion, 1)
        self.create_subscription(Odometry, '/state_estimation', self.odom, qos_profile_sensor_data)
        self.create_subscription(Imu, param('imu_topic', '/utlidar/transformed_raw_imu'), self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, param('lidar_topic', '/utlidar/transformed_cloud'), self.cloud, qos_profile_sensor_data)
        self.create_subscription(Path, '/path', self.path, 1)
        self.create_subscription(Int8, '/stop', self.stop, 1)
        if not self.local_route:
            self.create_subscription(String, '/go2/navigation/planner_heartbeat', self.active, 1)
        self.create_subscription(PointStamped, '/goal_point' if self.far_route else '/way_point', self.goal, 1)
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
        if valid:
            self.odom_history.observe(msg)
        else:
            self.odom_history.clear()
        self.gate.observe('odom', valid, time.monotonic(), self.header_age(msg))

    def imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.gate.observe('imu', self.header_valid(msg) and msg.header.frame_id == 'body' and all(math.isfinite(x) for x in (a.x,a.y,a.z,g.x,g.y,g.z)), time.monotonic(), self.header_age(msg))

    def cloud(self, msg):
        valid = self.header_valid(msg) and msg.header.frame_id == 'body' and valid_cloud_layout(msg)
        self.gate.observe('lidar', valid, time.monotonic(), self.header_age(msg))

    def path(self, msg):
        valid = self.header_valid(msg) and msg.header.frame_id == 'vehicle' and len(msg.poses) > 1 and all(math.isfinite(v) for p in msg.poses for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z))
        self.gate.observe('path', valid, time.monotonic(), self.header_age(msg))

    def goal(self, msg):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        valid = (not self.gate.cancelled and stamp > self.goal_after_ns and
                 self.header_valid(msg) and msg.header.frame_id in ('map', 'camera_init') and
                 all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)))
        if valid:
            self.gate.goal()

    def active(self, msg):
        try:
            if len(msg.data) > 1024 or self.active_owner_gid is None:
                return
            data = json.loads(msg.data)
            observed = data['observed_ns']
            session = data['session_id']
            now_ns = time.monotonic_ns()
            if (type(observed) is not int or not isinstance(session,str) or not 0 < len(session) <= 128 or
                type(data['active']) is not bool or observed <= self.active_owner_since_ns or
                observed <= self.active_observed_ns or not 0 <= now_ns-observed <= 500000000):
                return
            if self.active_session and session != self.active_session and now_ns-self.active_observed_ns <= 500000000:
                self.gate.planner_active(False, time.monotonic())
                return
            self.active_session = session
            self.active_observed_ns = observed
            self.active_hint = data['active']
            self.gate.planner_active(self.active_hint, observed / 1e9)
        except (ValueError,KeyError,TypeError):
            self.gate.planner_active(False, time.monotonic())

    def stop(self, msg):
        self.gate.observe('stop', msg.data == 0, time.monotonic())

    def cancel(self, request, response):
        self.gate.cancel()
        self.local_enabled = False
        if self.far_route:
            self.far_cancel_pending = True
            self.far_cancel_retry_at = 0.
        self.tick()
        response.success = True
        response.message = 'velocity revoked; cancel latched; resume requires a new request/path'
        return response

    def resume(self, request, response):
        if self.far_cancel_pending or self.far_cancel_future is not None:
            response.success = False
            response.message = 'FAR goal cancellation not acknowledged; velocity remains revoked'
            return response
        self.gate.resume()
        self.goal_after_ns = self.get_clock().now().nanoseconds
        self.local_enabled = True
        response.success = True
        response.message = 'cancel cleared; external stop retained; waiting for a new goal/path/request in stack mode'
        return response

    def cancel_far_tick(self, now):
        # One bounded in-flight RPC, serviced outside the leg process.
        if self.far_cancel_future is not None:
            future = self.far_cancel_future
            if future.done():
                try:
                    self.far_cancel_pending = not future.result().success
                except Exception:
                    self.far_cancel_pending = True
                self.far_cancel_future = None
                self.far_cancel_retry_at = now + .5
            elif now - self.far_cancel_started > 1.:
                self.far_cancel_client.remove_pending_request(future)
                self.far_cancel_future = None
                self.far_cancel_retry_at = now + .5
            return
        if not self.far_cancel_pending or now < self.far_cancel_retry_at:
            return
        self.far_cancel_retry_at = now + .5
        if self.far_cancel_client.service_is_ready():
            self.far_cancel_future = self.far_cancel_client.call_async(Trigger.Request())
            self.far_cancel_started = now

    def tick(self):
        now = time.monotonic()
        if self.gate.mode == 'stack' and now - self.last_tf_check >= .2:
            self.last_tf_check = now
            try:
                lookup = self.tf_buffer.lookup_transform('map','vehicle',Time())
                transform = lookup.transform
                stamp_ns = lookup.header.stamp.sec * 1000000000 + lookup.header.stamp.nanosec
                tf_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                time_valid = stamp_ns > 0 and 0 <= tf_age <= .5
                # Odom can arrive before its TF on a different DDS topic.
                # Compare a fresh sample at a time already available in TF;
                # do not wait in a callback or loosen the freshness deadline.
                odom = self.odom_history.at_or_before(stamp_ns)
                if odom is None or not self.header_valid(odom):
                    time_valid = False
                else:
                    otf = self.tf_buffer.lookup_transform(odom.header.frame_id, 'aft_mapped', Time.from_msg(odom.header.stamp)).transform
                    p, oq = odom.pose.pose.position, odom.pose.pose.orientation
                    t, tq = otf.translation, otf.rotation
                    position_error = max(abs(a-b) for a,b in zip((p.x,p.y,p.z),(t.x,t.y,t.z)))
                    qa, qb = (oq.x,oq.y,oq.z,oq.w), (tq.x,tq.y,tq.z,tq.w)
                    rotation_error = min(max(abs(a-b) for a,b in zip(qa,qb)),max(abs(a+b) for a,b in zip(qa,qb)))
                    time_valid = time_valid and position_error < .001 and rotation_error < .001
                self.tf_error = "" if time_valid else "tf_stale_static_or_odometry_mismatch"

                v, q = transform.translation, transform.rotation
                valid = time_valid and all(math.isfinite(x) for x in (v.x,v.y,v.z,q.x,q.y,q.z,q.w)) and .5 < sum(x*x for x in (q.x,q.y,q.z,q.w)) < 1.5
            except Exception as error:
                self.tf_error = str(error)
                valid = False
            self.gate.observe('tf',valid,now,max(0.,tf_age) if valid else 0.)
        self.cancel_far_tick(now)
        if self.local_route:
            self.active_hint = self.local_enabled and not self.gate.cancelled and self.gate.goal_received and self.gate.localization_ready(now)
            self.active_pub.publish(Bool(data=self.active_hint))
            self.gate.planner_active(self.active_hint, now)
        elif self.gate.mode == 'stack':
            if now - self.last_graph_check >= .2:
                owners = self.get_publishers_info_by_topic('/go2/navigation/planner_heartbeat')
                intents = self.get_publishers_info_by_topic('/navigation_active')
                same_owner = len(owners) == len(intents) == 1 and (owners[0].node_name, owners[0].node_namespace) == (intents[0].node_name, intents[0].node_namespace)
                gid = bytes(owners[0].endpoint_gid) if same_owner else None
                if gid != self.active_owner_gid:
                    self.active_hint = False
                    self.active_owner_since_ns = time.monotonic_ns()
                    self.active_session = ''
                    self.active_observed_ns = 0
                    self.gate.planner_active(False, now)
                self.active_owner_gid = gid
                self.last_graph_check = now
            if self.active_owner_gid is None:
                self.gate.planner_active(False, now)
        now = time.monotonic()  # graph/TF work must not renew an aged request
        value = self.gate.output(now)
        enabled = self.gate.enabled(now)
        age = (now - self.gate.request_received + self.gate.request_header_age) if value != (0.,0.,0.) else 0.
        ready = String(data=json.dumps(dict(observed_ns=time.monotonic_ns(), session_id=self.session, navigation_ready=self.gate.localization_ready(now), perception_ready=False, command_enabled=enabled, command=list(value), command_age_s=age)))
        self.ready_pub.publish(ready)
        msg = TwistStamped()
        msg.header.frame_id = 'vehicle'
        # Preserve request age rather than re-stamping an old nonzero command.
        msg.header.stamp = Time(nanoseconds=max(0, self.get_clock().now().nanoseconds - int(age * 1e9))).to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = value
        self.cmd_pub.publish(msg)
        self.status_pub.publish(String(data=json.dumps(dict(session_id=self.session, input_mode=self.gate.mode, navigation_ready=self.gate.localization_ready(now), command_enabled=enabled, cancel_latched=self.gate.cancelled, far_cancel_pending=self.far_cancel_pending or self.far_cancel_future is not None, new_goal_required=self.gate.mode=="stack" and not self.gate.goal_received, block_reason=self.gate.block_reason, tf_error=self.tf_error, health_ages={name: None if stamp is None else now-stamp for name,stamp in self.gate.health.items()}, odom_frame=self.last_odom_frame, command=list(value)))))

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
