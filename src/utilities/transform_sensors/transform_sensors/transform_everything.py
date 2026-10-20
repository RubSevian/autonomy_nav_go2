#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Vector3
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import numpy as np
import yaml
import json
import time
from copy import deepcopy
from std_msgs.msg import String
from .time_alignment import AlignmentConfig, SensorTimeAlignment
from .bounded_time_alignment import BoundedConfig, BoundedTimeAlignment, finite_imu_fields
from .point_records import PointRecords
from .imu_cdr import ImuCdr

import os
import signal
from rclpy.executors import ExternalShutdownException


# Quaternion convention in ROS messages is [x, y, z, w].  Keeping these small
# helpers local avoids undeclared pip dependencies (tf_transformations and
# transforms3d) on the Jetson image.
def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return np.array([-x, -y, -z, w])


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    norm = x * x + y * y + z * z + w * w
    if norm < np.finfo(float).eps:
        return np.eye(3)
    x, y, z, w = np.array([x, y, z, w]) / np.sqrt(norm)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

class Repuber(Node):
    def __init__(self):
        super().__init__('sensor_transformer')
        def param(name, default):
            return self.declare_parameter(name, default).value
        imu_depth=param('imu_subscription_depth',64)
        if type(imu_depth) is not int or imu_depth<1:raise ValueError('imu_subscription_depth must be positive integer')
        self.imu_transport_mode=param('imu_transport_mode','ros_message')
        if self.imu_transport_mode not in ('ros_message','projected_cdr'):raise ValueError('unknown imu_transport_mode')
        self.imu_cdr=ImuCdr()
        self.imu_sub = self.create_subscription(Imu, param('raw_imu_topic', '/utlidar/imu'), self.imu_callback,
            QoSProfile(depth=imu_depth,reliability=ReliabilityPolicy.BEST_EFFORT),raw=self.imu_transport_mode=='projected_cdr')
        self.cloud_sub = self.create_subscription(PointCloud2, param('raw_cloud_topic', '/utlidar/cloud'), self.cloud_callback, qos_profile_sensor_data)
        
        self.imu_raw_pub = self.create_publisher(Imu, param('raw_imu_output_topic', '/utlidar/transformed_raw_imu'), 50)
        self.imu_pub = self.create_publisher(Imu, param('legacy_imu_output_topic', '/utlidar/transformed_imu'), 50)
        self.cloud_pub = self.create_publisher(PointCloud2, param('cloud_topic', '/utlidar/transformed_cloud'), 50)

        self.imu_stationary_list = []
        
        variant = param('time_alignment_variant', 'bounded_common_v1')
        if variant not in ('fixed', 'bounded_common_v1'):
            raise ValueError('unknown time alignment variant')
        config_type, clock_type = ((BoundedConfig, BoundedTimeAlignment) if variant == 'bounded_common_v1'
                                   else (AlignmentConfig, SensorTimeAlignment))
        self.time_alignment = clock_type(config_type(
            **{key: param('time_' + key, value) for key, value in vars(config_type()).items()}))
        self.time_status_pub = self.create_publisher(String, param('time_status_topic', '/utlidar/time_alignment'), 1)
        self.create_timer(param('time_status_period_s', .1 if variant == 'bounded_common_v1' else 1.0), self.publish_time_status)
        self._time_state = self.time_alignment.state
        
        self.cam_offset = param('cam_offset_m', 0.046825)
        self.imu_tilt = param('imu_tilt_deg', 15.1) / 180 * 3.1415926

        self.declare_parameter('calibration_path', '')
        calibration_path = self.get_parameter('calibration_path').value
        calib_data = {
                'acc_bias_x': 0.0,
                'acc_bias_y': 0.0,
                'acc_bias_z': 0.0,
                'ang_bias_x': 0.0,
                'ang_bias_y': 0.0,
                'ang_bias_z': 0.0,
                'ang_z2x_proj': 0.15,
                'ang_z2y_proj': -0.28
            }
        if not calibration_path:
            calibration_path = os.path.join(os.path.expanduser('~'), 'Desktop/imu_calib_data.yaml')
        try:
            with open(calibration_path, 'r', encoding='utf-8') as calib_file:
                loaded_calib_data = yaml.safe_load(calib_file) or {}
            if not isinstance(loaded_calib_data, dict):
                raise ValueError('calibration YAML must contain a mapping')
            calib_data.update(loaded_calib_data)
            self.get_logger().info(f'Loaded IMU calibration: {calibration_path}')
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.get_logger().warn(f'Using default IMU calibration: {error}')
            
        self.acc_bias_x = calib_data['acc_bias_x']
        self.acc_bias_y = calib_data['acc_bias_y']
        self.acc_bias_z = calib_data['acc_bias_z']
        self.ang_bias_x = calib_data['ang_bias_x']
        self.ang_bias_y = calib_data['ang_bias_y']
        self.ang_bias_z = calib_data['ang_bias_z']
        self.ang_z2x_proj = calib_data['ang_z2x_proj']
        self.ang_z2y_proj = calib_data['ang_z2y_proj']
                
        self.body2cloud_trans = TransformStamped()
        self.body2cloud_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2cloud_trans.header.frame_id = "body"
        self.body2cloud_trans.child_frame_id = "utlidar_lidar_1"
        self.body2cloud_trans.transform.translation.x = 0.0
        self.body2cloud_trans.transform.translation.y = 0.0
        self.body2cloud_trans.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, param('cloud_pitch_rad', 2.87820258505555555556), 0)
        self.body2cloud_trans.transform.rotation.x = quat[0]
        self.body2cloud_trans.transform.rotation.y = quat[1]
        self.body2cloud_trans.transform.rotation.z = quat[2]
        self.body2cloud_trans.transform.rotation.w = quat[3]
        
        self.body2imu_trans = TransformStamped()
        self.body2imu_trans.header.stamp = self.get_clock().now().to_msg()
        self.body2imu_trans.header.frame_id = "body"
        self.body2imu_trans.child_frame_id = "utlidar_imu_1"
        self.body2imu_trans.transform.translation.x = 0.0
        self.body2imu_trans.transform.translation.y = 0.0
        self.body2imu_trans.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, param('imu_pitch_rad', 2.87820258505555555556), param('imu_yaw_rad', 3.14159265358))
        self.body2imu_trans.transform.rotation.x = quat[0]
        self.body2imu_trans.transform.rotation.y = quat[1]
        self.body2imu_trans.transform.rotation.z = quat[2]
        self.body2imu_trans.transform.rotation.w = quat[3]
        # Geometry and calibration are constructor-owned for this session.
        self._cloud_rotation = quaternion_matrix(np.array([
            self.body2cloud_trans.transform.rotation.x,
            self.body2cloud_trans.transform.rotation.y,
            self.body2cloud_trans.transform.rotation.z,
            self.body2cloud_trans.transform.rotation.w,
        ]))
        self._cloud_translation = np.array([
            self.body2cloud_trans.transform.translation.x,
            self.body2cloud_trans.transform.translation.y,
            self.body2cloud_trans.transform.translation.z,
        ])
        self._imu_rotation = np.array([
            self.body2imu_trans.transform.rotation.x,
            self.body2imu_trans.transform.rotation.y,
            self.body2imu_trans.transform.rotation.z,
            self.body2imu_trans.transform.rotation.w,
        ])
        self._imu_cos, self._imu_sin = np.cos(self.imu_tilt), np.sin(self.imu_tilt)
        
        minimum = param('self_filter_min_m', [-0.7, -0.3, -0.6])
        maximum = param('self_filter_max_m', [-0.1, 0.3, 0.0])
        self.x_filter_min, self.y_filter_min, self.z_filter_min = minimum
        self.x_filter_max, self.y_filter_max, self.z_filter_max = maximum
        self.z_filter_min -= self.cam_offset
        self.z_filter_max -= self.cam_offset

    def is_in_filter_box(self, point):
        # Check if the point is in the filter box
        is_in_box = point[0] > self.x_filter_min and \
                    point[0] < self.x_filter_max and \
                    point[1] > self.y_filter_min and \
                    point[1] < self.y_filter_max and \
                    point[2] > self.z_filter_min and \
                    point[2] < self.z_filter_max
        return is_in_box

    def cloud_callback(self, data):
        receive_ros_ns = self.get_clock().now().nanoseconds
        receive_mono_ns = time.monotonic_ns()
        points = pc2.read_points(data)
        if points.dtype.names is None or not {'x', 'y', 'z', 'time'}.issubset(points.dtype.names):
            self.get_logger().error('Dropping cloud with no x/y/z/time point fields')
            return
        point_times = points['time']
        if not np.isfinite(point_times).all() or (point_times < 0).any():
            self.get_logger().error('Dropping cloud with invalid relative point times')
            return
        span_ns = int(np.ceil(float(point_times.max()) * 1e9)) if points.size else 0
        mapped_ns = self.map_sensor_stamp('cloud', data.header.stamp, receive_ros_ns, receive_mono_ns, span_ns)
        if mapped_ns is None:
            return
        header = deepcopy(data.header)
        header.stamp = Time(nanoseconds=mapped_ns).to_msg()
        header.frame_id = 'body'
        if points.size == 0:
            elevated_cloud = pc2.create_cloud(header, data.fields, [], point_step=data.point_step)
            self.cloud_pub.publish(elevated_cloud)
            return

        # Keep the structured PointCloud2 dtype intact: LiDAR ring/time fields
        # must not be coerced to a common float array by NumPy.
        # Copy complete records. A structured NumPy copy/index operation only
        # copies named fields and can leave padding uninitialized. Unitree/PCL
        # records include homogeneous-coordinate bytes outside PointFields.
        records = PointRecords(points)
        transformed_points = records.points
        if isinstance(self.time_alignment, BoundedTimeAlignment):
            source_ns = data.header.stamp.sec * 1000000000 + data.header.stamp.nanosec
            transformed_points['time'] = self.time_alignment.map_relative_times(source_ns, point_times)
        xyz = np.column_stack((points['x'], points['y'], points['z']))
        valid_xyz = np.isfinite(xyz).all(axis=1)
        xyz = xyz @ self._cloud_rotation.T + self._cloud_translation
        xyz[:, 2] -= self.cam_offset
        transformed_points['x'] = xyz[:, 0]
        transformed_points['y'] = xyz[:, 1]
        transformed_points['z'] = xyz[:, 2]
        in_filter_box = (
            (transformed_points['x'] > self.x_filter_min) &
            (transformed_points['x'] < self.x_filter_max) &
            (transformed_points['y'] > self.y_filter_min) &
            (transformed_points['y'] < self.y_filter_max) &
            (transformed_points['z'] > self.z_filter_min) &
            (transformed_points['z'] < self.z_filter_max)
        )
        transformed_points = records.select(valid_xyz & ~in_filter_box)
        
        # read_points retains the input record size, including trailing padding.
        # Unitree points can occupy 32 bytes even when the last field ends at 28.
        elevated_cloud = pc2.create_cloud(
            header, data.fields, transformed_points, point_step=data.point_step)
        elevated_cloud.is_dense = bool(data.is_dense and valid_xyz.all())

        self.cloud_pub.publish(elevated_cloud)

    def publish_time_status(self):
        if isinstance(self.time_alignment, BoundedTimeAlignment):
            self.time_alignment.check_health(time.monotonic_ns())
        snapshot = self.time_alignment.status()
        snapshot['imu_transport']=dict(mode=self.imu_transport_mode,**self.imu_cdr.status())
        snapshot['snapshot_mono_ns'] = time.monotonic_ns()
        self.time_status_pub.publish(String(data=json.dumps(snapshot)))

    def map_sensor_stamp(self, stream, stamp, receive_ros_ns, receive_mono_ns, span_ns=0):
        return self.map_sensor_values(stream,stamp.sec,stamp.nanosec,receive_ros_ns,receive_mono_ns,span_ns)

    def map_sensor_values(self, stream, sec, nanosec, receive_ros_ns, receive_mono_ns, span_ns=0):
        source_ns = sec * 1000000000 + nanosec
        if sec < 0 or not 0 <= nanosec < 1000000000:
            source_ns = 0
        mapped = self.time_alignment.map_stamp(stream, source_ns, receive_ros_ns, receive_mono_ns, span_ns)
        if self.time_alignment.state != self._time_state:
            self._time_state = self.time_alignment.state
            message = 'Sensor time alignment: ' + json.dumps(self.time_alignment.status())
            if self._time_state == 'FAULT':
                self.get_logger().error(message + '; restart localization after checking clocks')
            else:
                self.get_logger().info(message)
        return mapped
            
    def transform_vector(self, vector, rotation):
        # Transform a vector using a given quaternion rotation
        q_vector = [vector.x, vector.y, vector.z, 0.0]
        q_rotated = quaternion_multiply(
            quaternion_multiply(rotation, q_vector),
            quaternion_conjugate(rotation)
        )
        
        ret_vec = Vector3()
        ret_vec.x = q_rotated[0]
        ret_vec.y = q_rotated[1]
        ret_vec.z = q_rotated[2]
        return ret_vec


    def imu_callback(self, data):
        sec,nanosec,orientation,angular,acceleration=self.imu_cdr.read(data)
        if isinstance(self.time_alignment, BoundedTimeAlignment):
            values = angular + acceleration + orientation
            if not finite_imu_fields(values):
                self.time_alignment._drop('imu', 'invalid_imu_fields')
                return
        mapped_ns = self.map_sensor_values('imu', sec, nanosec,
                                          self.get_clock().now().nanoseconds, time.monotonic_ns())
        if mapped_ns is None:
            return
        transformed_orientation = quaternion_multiply(
            self._imu_rotation, orientation
        )
        
        x = angular[0]
        y = -angular[1]
        z = -angular[2]
        

        x2 = self._imu_cos * x - self._imu_sin * z
        y2 = y
        z2 = self._imu_sin * x + self._imu_cos * z

        x2 -= self.ang_bias_x
        y2 -= self.ang_bias_y
        z2 -= self.ang_bias_z
        
        x_comp_rate = self.ang_z2x_proj
        y_comp_rate = self.ang_z2y_proj
        
        x2 += x_comp_rate * z2
        y2 += y_comp_rate * z2
        
        
        acc_x = acceleration[0]
        acc_y = -acceleration[1]
        acc_z = -acceleration[2]
        
        acc_x2 = self._imu_cos * acc_x - self._imu_sin * acc_z
        acc_y2 = acc_y
        acc_z2 = self._imu_sin * acc_x + self._imu_cos * acc_z
        calibrated_acc=(acc_x2-self.acc_bias_x,acc_y2-self.acc_bias_y,acc_z2-self.acc_bias_z)
        if self.imu_transport_mode=='projected_cdr' and self.imu_cdr.template is not None:
            self.imu_raw_pub.publish(self.imu_cdr.encode(mapped_ns,transformed_orientation,(x2,y2,z2),calibrated_acc))
            self.imu_pub.publish(self.imu_cdr.encode(mapped_ns,(0.,0.,0.,1.),(x2,y2,z2),(0.,0.,0.)))
            return
        transformed_angular_velocity = Vector3()
        transformed_angular_velocity.x = x2
        transformed_angular_velocity.y = y2
        transformed_angular_velocity.z = z2
        transformed_linear_acceleration = Vector3()
        transformed_linear_acceleration.x = calibrated_acc[0]
        transformed_linear_acceleration.y = calibrated_acc[1]
        transformed_linear_acceleration.z = calibrated_acc[2]
        

        transformed_imu = Imu()
        transformed_imu.header.stamp = Time(nanoseconds=mapped_ns).to_msg()
        transformed_imu.header.frame_id = 'body'
        transformed_imu.orientation.x = transformed_orientation[0]
        transformed_imu.orientation.y = transformed_orientation[1]
        transformed_imu.orientation.z = transformed_orientation[2]
        transformed_imu.orientation.w = transformed_orientation[3]
        transformed_imu.angular_velocity = transformed_angular_velocity
        transformed_imu.linear_acceleration = transformed_linear_acceleration
        
        self.imu_raw_pub.publish(transformed_imu)
        
        transformed_imu.orientation.x = 0.0
        transformed_imu.orientation.y = 0.0
        transformed_imu.orientation.z = 0.0
        transformed_imu.orientation.w = 1.0
        
        transformed_imu.linear_acceleration.x = 0.0
        transformed_imu.linear_acceleration.y = 0.0
        transformed_imu.linear_acceleration.z = 0.0
        
        self.imu_pub.publish(transformed_imu)

def main(args=None):
    rclpy.init(args=args)
    transform_node = Repuber()
    try:
        rclpy.spin(transform_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        transform_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
