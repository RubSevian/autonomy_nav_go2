"""Verified Imu CDR projection/encoding; ROS decoding is the fallback.

Only standard little-endian CDR with the installed Imu schema is projected.
The output template comes from the installed ROS serializer, not an SDK struct.
Covariances remain zero, exactly as the existing calibrated relay publishes.
"""
import struct
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Imu


class ImuCdr:
    def __init__(self):
        self.projected = self.decoded = 0
        self.template = None
        self.output_start = None
        schema = dict(header='std_msgs/Header', orientation='geometry_msgs/Quaternion',
            orientation_covariance='double[9]', angular_velocity='geometry_msgs/Vector3',
            angular_velocity_covariance='double[9]', linear_acceleration='geometry_msgs/Vector3',
            linear_acceleration_covariance='double[9]')
        sample = Imu()
        if (Imu.get_fields_and_field_types() != schema
                or sample.header.get_fields_and_field_types() != dict(stamp='builtin_interfaces/Time', frame_id='string')
                or sample.header.stamp.get_fields_and_field_types() != dict(sec='int32', nanosec='uint32')
                or sample.orientation.get_fields_and_field_types() != dict(x='double', y='double', z='double', w='double')
                or sample.angular_velocity.get_fields_and_field_types() != dict(x='double', y='double', z='double')
                or sample.linear_acceleration.get_fields_and_field_types() != dict(x='double', y='double', z='double')):
            return
        # Prove offsets/alignment against the actual serializer for different
        # UTF-8 string sizes. No native ABI or Point-LIO field offsets assumed.
        values = (1001.125, 1002.25, 1003.375, 1004.5, 1005.625, 1006.75, 1007.875, 1008.125, 1009.25, 1010.375)
        for frame in ('', 'body', 'frame_with_padding', 'камера'):
            sample.header.frame_id = frame
            sample.header.stamp.sec = 0x13572468
            sample.header.stamp.nanosec = 0x24681357
            for obj, axes, chunk in ((sample.orientation, 'xyzw', values[:4]),
                    (sample.angular_velocity, 'xyz', values[4:7]),
                    (sample.linear_acceleration, 'xyz', values[7:])):
                for axis, value in zip(axes, chunk):setattr(obj, axis, value)
            wire = serialize_message(sample)
            start = self._start(wire)
            if (start is None or struct.unpack_from('<iI', wire, 4) != (sample.header.stamp.sec, sample.header.stamp.nanosec)
                    or struct.unpack_from('<4d', wire, start) != values[:4]
                    or struct.unpack_from('<3d', wire, start+104) != values[4:7]
                    or struct.unpack_from('<3d', wire, start+200) != values[7:]):
                return
        output = Imu();output.header.frame_id = 'body'
        template = bytearray(serialize_message(output));start = self._start(template)
        if start is None:return
        end_string = 16+struct.unpack_from('<I',template,12)[0]
        template[end_string:start] = bytes(start-end_string)
        self.template, self.output_start = bytes(template), start

    @staticmethod
    def _start(data):
        if len(data)<20 or data[:4]!=b'\x00\x01\x00\x00':return None
        size = struct.unpack_from('<I',data,12)[0]
        if size<1 or size>len(data)-16 or data[16+size-1]!=0:return None
        try:bytes(data[16:16+size-1]).decode('utf-8')
        except UnicodeDecodeError:return None
        start = 4+((16+size-4+7)//8)*8
        return start if len(data)==start+296 else None

    def read(self, data):
        start = self._start(data) if self.template is not None and isinstance(data,bytes) else None
        if start is not None:
            self.projected += 1
            sec,nsec = struct.unpack_from('<iI',data,4)
            return sec,nsec,struct.unpack_from('<4d',data,start),struct.unpack_from('<3d',data,start+104),struct.unpack_from('<3d',data,start+200)
        self.decoded += 1
        msg = data if isinstance(data,Imu) else deserialize_message(data,Imu)
        return (msg.header.stamp.sec,msg.header.stamp.nanosec,
            (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),
            (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z),
            (msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z))

    def encode(self, mapped_ns, orientation, angular, acceleration):
        if self.template is None:raise RuntimeError('Verified Imu CDR layout unavailable')
        sec,nsec = divmod(mapped_ns,1_000_000_000)
        data = bytearray(self.template)
        struct.pack_into('<iI',data,4,sec,nsec)
        struct.pack_into('<4d',data,self.output_start,*orientation)
        struct.pack_into('<3d',data,self.output_start+104,*angular)
        struct.pack_into('<3d',data,self.output_start+200,*acceleration)
        return bytes(data)

    def status(self):
        return dict(verified=self.template is not None,projected=self.projected,ros_decoded=self.decoded)
