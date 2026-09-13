#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Vector3
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import yaml

import os


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
        self.imu_sub = self.create_subscription(Imu, '/utlidar/imu', self.imu_callback, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(PointCloud2, '/utlidar/cloud', self.cloud_callback, qos_profile_sensor_data)
        
        self.imu_raw_pub = self.create_publisher(Imu, '/utlidar/transformed_raw_imu', 50)
        self.imu_pub = self.create_publisher(Imu, '/utlidar/transformed_imu', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/utlidar/transformed_cloud', 50)

        self.imu_stationary_list = []
        
        self.time_stamp_offset = 0
        self.time_stamp_offset_set = False
        
        self.cam_offset = 0.046825

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
        quat = quaternion_from_euler(0, 2.87820258505555555556, 0)
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
        quat = quaternion_from_euler(0, 2.87820258505555555556, 3.14159265358)
        self.body2imu_trans.transform.rotation.x = quat[0]
        self.body2imu_trans.transform.rotation.y = quat[1]
        self.body2imu_trans.transform.rotation.z = quat[2]
        self.body2imu_trans.transform.rotation.w = quat[3]
        
        self.x_filter_min = -0.7
        self.x_filter_max = -0.1
        self.y_filter_min = -0.3
        self.y_filter_max = 0.3
        self.z_filter_min = -0.6 - self.cam_offset
        self.z_filter_max = 0 - self.cam_offset

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
        if not self.time_stamp_offset_set:
            self.time_stamp_offset = self.get_clock().now().nanoseconds - Time.from_msg(data.header.stamp).nanoseconds
            self.time_stamp_offset_set = True
                
        cloud_arr = pc2.read_points_list(data)
        if not cloud_arr:
            elevated_cloud = pc2.create_cloud(data.header, data.fields, [])
            elevated_cloud.header.frame_id = 'body'
            elevated_cloud.header.stamp = Time(
                nanoseconds=Time.from_msg(data.header.stamp).nanoseconds + self.time_stamp_offset
            ).to_msg()
            self.cloud_pub.publish(elevated_cloud)
            return
        points = np.asarray(cloud_arr)
        if points.ndim != 2 or points.shape[1] < 3:
            self.get_logger().error('Dropping cloud with no x/y/z point fields')
            return

        transform = self.body2cloud_trans.transform
        mat = quaternion_matrix(np.array([
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w,
        ]))
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        
        transformed_points = points.copy()
        transformed_points[:, 0:3] = points[:, 0:3] @ mat.T + translation
        transformed_points[:, 2] -= self.cam_offset
        in_filter_box = (
            (transformed_points[:, 0] > self.x_filter_min) &
            (transformed_points[:, 0] < self.x_filter_max) &
            (transformed_points[:, 1] > self.y_filter_min) &
            (transformed_points[:, 1] < self.y_filter_max) &
            (transformed_points[:, 2] > self.z_filter_min) &
            (transformed_points[:, 2] < self.z_filter_max)
        )
        transformed_points = transformed_points[~in_filter_box].tolist()
        
        elevated_cloud = pc2.create_cloud(data.header, data.fields, transformed_points)
        elevated_cloud.header.stamp = Time(nanoseconds=Time.from_msg(elevated_cloud.header.stamp).nanoseconds + self.time_stamp_offset).to_msg()
        elevated_cloud.header.frame_id = "body"
        elevated_cloud.is_dense = data.is_dense

        self.cloud_pub.publish(elevated_cloud)
            
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
        # Both streams use the same offset.  Dropping early IMU packets avoids
        # mixing two time bases before the first LiDAR stamp establishes it.
        if not self.time_stamp_offset_set:
            return
        trans = np.zeros(3)
        trans[0] = self.body2imu_trans.transform.translation.x
        trans[1] = self.body2imu_trans.transform.translation.y
        trans[2] = self.body2imu_trans.transform.translation.z
        
        rot = np.zeros(4)
        rot[0] = self.body2imu_trans.transform.rotation.x
        rot[1] = self.body2imu_trans.transform.rotation.y
        rot[2] = self.body2imu_trans.transform.rotation.z
        rot[3] = self.body2imu_trans.transform.rotation.w
        
        transformed_orientation = quaternion_multiply(
            rot, [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        )
        
        x = data.angular_velocity.x
        y = -data.angular_velocity.y
        z = -data.angular_velocity.z
        
        theta = 15.1 / 180 * 3.1415926

        x2 = np.cos(theta) * x - np.sin(theta) * z
        y2 = y
        z2 = np.sin(theta) * x + np.cos(theta) * z

        x2 -= self.ang_bias_x
        y2 -= self.ang_bias_y
        z2 -= self.ang_bias_z
        
        x_comp_rate = self.ang_z2x_proj
        y_comp_rate = self.ang_z2y_proj
        
        x2 += x_comp_rate * z2
        y2 += y_comp_rate * z2
        
        transformed_angular_velocity = Vector3()
        transformed_angular_velocity.x = x2
        transformed_angular_velocity.y = y2
        transformed_angular_velocity.z = z2
        
        acc_x = data.linear_acceleration.x
        acc_y = -data.linear_acceleration.y
        acc_z = -data.linear_acceleration.z
        
        acc_x2 = np.cos(theta) * acc_x - np.sin(theta) * acc_z
        acc_y2 = acc_y
        acc_z2 = np.sin(theta) * acc_x + np.cos(theta) * acc_z
        transformed_linear_acceleration = Vector3()
        transformed_linear_acceleration.x = acc_x2 - self.acc_bias_x
        transformed_linear_acceleration.y = acc_y2 - self.acc_bias_y
        transformed_linear_acceleration.z = acc_z2 - self.acc_bias_z
        

        transformed_imu = Imu()
        transformed_imu.header.stamp = data.header.stamp
        transformed_imu.header.frame_id = 'body'
        transformed_imu.orientation.x = transformed_orientation[0]
        transformed_imu.orientation.y = transformed_orientation[1]
        transformed_imu.orientation.z = transformed_orientation[2]
        transformed_imu.orientation.w = transformed_orientation[3]
        transformed_imu.angular_velocity = transformed_angular_velocity
        transformed_imu.linear_acceleration = transformed_linear_acceleration
        
        transformed_imu.header.stamp = Time(nanoseconds=Time.from_msg(transformed_imu.header.stamp).nanoseconds + self.time_stamp_offset).to_msg()
        
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
    finally:
        transform_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
