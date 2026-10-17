#!/usr/bin/env python3
"""Readiness/cancel/velocity adapter. Never imports Unitree SDK or arm SDK."""
import json
import math
import time
import uuid
import signal
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String, Bool, Int8
from std_srvs.srv import Trigger
from navigation_gate import NavigationGate
from diagnostic_msgs.msg import DiagnosticArray
from navigation_goal import GoalTracker
from navigation_contract import valid_cloud_layout, locomotion_ready
from localization_pairing import LocalizationPairing, PairingConfig, PoseSample, PairLookupError, pose_errors
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time

def odom_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return PoseSample(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                      msg.header.frame_id, msg.child_frame_id,
                      (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def transform_pose(msg):
    p, q = msg.transform.translation, msg.transform.rotation
    return PoseSample(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                      msg.header.frame_id, msg.child_frame_id,
                      (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class LocalizationBuffer(Buffer):
    """Capture actual dynamic revisions in the same callback that updates TF.

    No extra DDS subscription, queue, thread or interpolation-generated sample.
    Static transforms continue through Buffer.set_transform_static unchanged.
    """
    def __init__(self, pairing):
        super().__init__()
        self.pairing = pairing

    def set_transform(self, transform, authority):
        super().set_transform(transform, authority)
        if transform.child_frame_id == 'aft_mapped' and transform.header.frame_id in ('map', 'camera_init'):
            self.pairing.observe_tf(transform_pose(transform))


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
        self.gate = NavigationGate(mode, param('command_timeout_s', .25), param('sensor_timeout_s', .5),
                                   tuple(param(key, default) for key,default in zip(('vx_limit','vy_limit','wz_limit'),(.2,.1,.1))))
        self.gate.clock_required = param('require_clock_alignment', False)
        if self.gate.clock_required:
            self.create_subscription(String, param('time_status_topic', '/utlidar/time_alignment'), self.alignment_status, 1)
        self.goals = GoalTracker(param('goal_tolerance_m', .25), param('goal_dwell_s', .5))
        self.tick_period = param('tick_period_s', .02)
        self.status_period = param('status_period_s', .2)
        self.tf_check_period = param('tf_check_period_s', .2)
        self.graph_check_period = param('graph_check_period_s', .2)
        self.far_status_timeout = param('far_status_timeout_s', 1.5)
        self.cancel_timeout = param('cancel_timeout_s', 1.)
        self.cancel_retry = param('cancel_retry_s', .5)
        if any(not math.isfinite(v) or v <= 0 for v in (self.tick_period, self.status_period, self.tf_check_period, self.graph_check_period, self.far_status_timeout, self.cancel_timeout, self.cancel_retry)):
            raise ValueError('Bridge periods/timeouts must be finite and positive')
        self.last_status_publish = 0.
        self.goal_tf_valid = False
        self.goal_after_ns = self.get_clock().now().nanoseconds
        self.far_cancel_pending = False
        self.far_cancel_future = None
        self.far_cancel_started = 0.
        self.far_cancel_retry_at = 0.
        self.far_cancel_client = self.create_client(Trigger, '/far_planner/cancel_goal') if self.far_route else None
        self.pairing = LocalizationPairing(PairingConfig(
            max_age_s=self.gate.sensor_timeout,
            history_s=param('tf_pair_history_s', 1.),
            max_samples=param('tf_pair_max_samples', 2048),
            position_tolerance_m=param('tf_pair_position_tolerance_m', .001),
            quaternion_tolerance=param('tf_pair_quaternion_tolerance', .001)))
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
        self.tf_buffer = LocalizationBuffer(self.pairing)
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
        if self.far_route:
            self.create_subscription(DiagnosticArray, '/far/planner_status', self.far_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if not self.local_route:
            self.create_subscription(String, '/go2/navigation/planner_heartbeat', self.active, 1)
        self.create_subscription(PointStamped, '/goal_point' if self.far_route else '/way_point', self.goal, 1)
        self.create_service(Trigger, '/go2/mission/cancel_navigation', self.cancel)
        self.create_service(Trigger, '/go2/mission/resume_navigation', self.resume)
        self.gate.observe('stop', True, time.monotonic())
        self.create_timer(self.tick_period, self.tick)
        self.get_logger().info('NAV request adapter mode=' + mode + '; no locomotion/Sport ownership')

    def header_age(self, msg):
        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000 or (stamp.sec == 0 and stamp.nanosec == 0):
            return float('inf')
        return (self.get_clock().now().nanoseconds - stamp.sec * 1000000000 - stamp.nanosec) / 1e9

    def alignment_status(self, msg):
        now = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
            stamp = data['snapshot_mono_ns']
            valid = (type(stamp) is int and data.get('variant') == 'bounded_common_v1'
                     and data.get('mapping_valid') is True and data.get('restart_required') is False)
            age = (now - stamp) / 1e9 if type(stamp) is int else float('inf')
            self.gate.observe_alignment(data.get('state'), valid, now / 1e9, age)
        except (ValueError, TypeError, KeyError):
            self.gate.observe_alignment('INVALID', False, now / 1e9)

    def header_valid(self, msg):
        age = self.header_age(msg)
        return bool(msg.header.frame_id) and math.isfinite(age) and 0 <= age <= self.gate.sensor_timeout

    def command(self, msg):
        if self.gate.mode == 'stack' and msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec < self.goals.stamp_ns:
            return
        self.gate.accept((msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z), self.header_age(msg), msg.header.frame_id, time.monotonic())

    def locomotion(self, msg):
        self.gate.locomotion(locomotion_ready(msg.data), time.monotonic())

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        valid = self.header_valid(msg) and msg.header.frame_id in ('map','camera_init') and msg.child_frame_id == 'aft_mapped' and all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and .5 < sum(x*x for x in (q.x,q.y,q.z,q.w)) < 1.5
        self.last_odom_frame = msg.header.frame_id
        if valid:
            self.pairing.observe_odom(odom_pose(msg))
        else:
            self.pairing.clear('invalid_odometry')
        self.gate.observe('odom', valid, time.monotonic(), self.header_age(msg))

    def imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.gate.observe('imu', self.header_valid(msg) and msg.header.frame_id == 'body' and all(math.isfinite(x) for x in (a.x,a.y,a.z,g.x,g.y,g.z)), time.monotonic(), self.header_age(msg))

    def cloud(self, msg):
        valid = self.header_valid(msg) and msg.header.frame_id == 'body' and valid_cloud_layout(msg)
        self.gate.observe('lidar', valid, time.monotonic(), self.header_age(msg))

    def path(self, msg):
        valid = (self.header_valid(msg) and msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec > self.goals.stamp_ns) and msg.header.frame_id == 'vehicle' and len(msg.poses) > 1 and all(math.isfinite(v) for p in msg.poses for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z))
        self.gate.observe('path', valid, time.monotonic(), self.header_age(msg))

    def goal(self, msg):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        valid = (not self.gate.cancelled and stamp > self.goal_after_ns and
                 self.header_valid(msg) and msg.header.frame_id in ('map', 'camera_init') and
                 all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)))
        if valid and self.goals.accept((msg.point.x,msg.point.y,msg.point.z), msg.header.frame_id, stamp, time.monotonic()):
            self.gate.clear_request()
            self.gate.health['path'] = None
            self.gate.goal()

    def far_status(self, msg):
        age = self.header_age(msg)
        if not self.goal_tf_valid or not self.gate.localization_ready(time.monotonic()) or not math.isfinite(age) or not 0 <= age <= self.far_status_timeout:
            return
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        for status in msg.status:
            if status.name == 'far_planner':
                if self.goals.far_status({v.key:v.value for v in status.values}, stamp, time.monotonic()):
                    self.gate.goal_received = False
                    self.gate.clear_request()

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
                observed <= self.active_observed_ns or not 0 <= now_ns-observed <= self.gate.sensor_timeout * 1e9):
                return
            if self.active_session and session != self.active_session and now_ns-self.active_observed_ns <= self.gate.sensor_timeout * 1e9:
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
        self.goals.finish('CANCELLED', time.monotonic())
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
        self.goals.reset()
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
                self.far_cancel_retry_at = now + self.cancel_retry
            elif now - self.far_cancel_started > self.cancel_timeout:
                self.far_cancel_client.remove_pending_request(future)
                self.far_cancel_future = None
                self.far_cancel_retry_at = now + self.cancel_retry
            return
        if not self.far_cancel_pending or now < self.far_cancel_retry_at:
            return
        self.far_cancel_retry_at = now + self.cancel_retry
        if self.far_cancel_client.service_is_ready():
            self.far_cancel_future = self.far_cancel_client.call_async(Trigger.Request())
            self.far_cancel_started = now

    def resolve_vehicle(self, odom, received_tf):
        """Validate the available TF revision, then resolve the same-time chain."""
        stamp = Time(nanoseconds=odom.stamp_ns)
        try:
            buffered = transform_pose(self.tf_buffer.lookup_transform(odom.frame, odom.child, stamp))
            pe, qe = pose_errors(buffered, received_tf)
            odom_pe, odom_qe = pose_errors(buffered, odom)
            config = self.pairing.config
            if (buffered.stamp_ns != odom.stamp_ns or pe >= config.position_tolerance_m or
                    qe >= config.quaternion_tolerance or odom_pe >= config.position_tolerance_m or
                    odom_qe >= config.quaternion_tolerance):
                raise PairLookupError('tf_buffer_revision_mismatch')
            vehicle = transform_pose(self.tf_buffer.lookup_transform('map', 'vehicle', stamp))
        except TransformException as error:
            raise PairLookupError('tf_chain_unavailable', str(error)) from error
        return vehicle

    def tick(self):
        now = time.monotonic()
        if self.gate.mode == 'stack' and now - self.last_tf_check >= self.tf_check_period:
            self.goal_tf_valid = False
            self.last_tf_check = now
            pair = self.pairing.select(self.get_clock().now().nanoseconds, self.resolve_vehicle)
            # Charge every tick against the ORIGINAL measurement time, including
            # fallback and ticks between TF checks. Reuse cannot renew freshness.
            tf_age = ((self.get_clock().now().nanoseconds - pair.odom.stamp_ns) / 1e9
                      if pair else float('inf'))
            valid = pair is not None and 0 <= tf_age <= self.gate.sensor_timeout
            self.tf_error = "" if valid else (self.pairing.reason or 'pair_expired')
            self.gate.observe('tf', valid, time.monotonic(), tf_age if valid else 0.)
            self.goal_tf_valid = valid
            if not valid:
                self.goals.inside_since = None
            if valid:
                x, y, _ = pair.vehicle.position
                self.goals.position(x, y, now, allow_local_completion=self.local_route and self.gate.locomotion_ready and self.gate.localization_ready(now))
                if self.goals.state == 'SUCCEEDED':
                    self.gate.goal_received = False
                    self.gate.clear_request()
        # The matching search runs at tf_check_period_s, but expiry is checked
        # at every envelope tick. No extra grace period until the next search.
        selected = self.pairing.selected
        if self.gate.mode == 'stack' and selected is not None:
            age = (self.get_clock().now().nanoseconds-selected.odom.stamp_ns)/1e9
            if not 0 <= age <= self.gate.sensor_timeout:
                self.tf_error = 'pair_expired' if age > 0 else 'pair_in_future'
                self.goal_tf_valid = False
                self.goals.inside_since = None
                self.gate.observe('tf', False, time.monotonic())
        self.cancel_far_tick(now)
        if self.local_route:
            self.active_hint = self.local_enabled and not self.gate.cancelled and self.gate.goal_received and self.gate.localization_ready(now)
            self.active_pub.publish(Bool(data=self.active_hint))
            self.gate.planner_active(self.active_hint, now)
        elif self.gate.mode == 'stack':
            if now - self.last_graph_check >= self.graph_check_period:
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
        if self.active_hint and self.gate.goal_received:
            self.goals.active()
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
        if now - self.last_status_publish < self.status_period:
            return
        self.last_status_publish = now
        self.status_pub.publish(String(data=json.dumps(dict(goal=self.goals.snapshot(now), session_id=self.session, input_mode=self.gate.mode, navigation_ready=self.gate.localization_ready(now), command_enabled=enabled, cancel_latched=self.gate.cancelled, far_cancel_pending=self.far_cancel_pending or self.far_cancel_future is not None, new_goal_required=self.gate.mode=="stack" and not self.gate.goal_received, block_reason=self.gate.block_reason, tf_error=self.tf_error, tf_pair=self.pairing.snapshot(self.get_clock().now().nanoseconds), health_ages={name: None if stamp is None else now-stamp for name,stamp in self.gate.health.items()}, odom_frame=self.last_odom_frame, command=list(value)))))

def main():
    # Stop spin in the Python thread before destroying DDS handles. The default
    # deferred ROS signal handler can invalidate them during take_message().
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*unused):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = NavigationBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        # A launch SIGINT can reach the child both through the terminal group
        # and forwarding. Do not interrupt resource destruction a second time.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
