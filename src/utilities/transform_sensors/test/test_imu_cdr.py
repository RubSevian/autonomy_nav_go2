"""Wire parity with installed ROS serialization; no node, DDS or hardware."""
import unittest,struct,math
from rclpy.serialization import serialize_message,deserialize_message
from sensor_msgs.msg import Imu
from transform_sensors.imu_cdr import ImuCdr

class ImuCdrTests(unittest.TestCase):
 def setUp(self):self.codec=ImuCdr();self.assertIsNotNone(self.codec.template)
 def message(self,frame):
  m=Imu();m.header.stamp.sec=1792441000;m.header.stamp.nanosec=123456789;m.header.frame_id=frame
  m.orientation.x=.1;m.orientation.y=-.2;m.orientation.z=.3;m.orientation.w=.9
  m.angular_velocity.x=-4.;m.angular_velocity.y=5.;m.angular_velocity.z=-6.
  m.linear_acceleration.x=7.;m.linear_acceleration.y=-8.;m.linear_acceleration.z=9.81
  m.orientation_covariance=[float(i) for i in range(9)]
  return m
 def test_projection_matches_decoder_for_variable_utf8_headers(self):
  for frame in ['', 'a','body','utlidar_imu_1','камера','x'*300]:
   m=self.message(frame);expected=self.codec.read(m);actual=self.codec.read(serialize_message(m));self.assertEqual(actual,expected)
  self.assertEqual(self.codec.projected,6)
 def test_output_is_standard_imu_with_existing_zero_covariance_contract(self):
  ns=1792441000123456789;ori=(.2,.3,-.4,.5);g=(1.,-2.,3.);acc=(-4.,5.,-6.)
  actual=deserialize_message(self.codec.encode(ns,ori,g,acc),Imu);expected=Imu();expected.header.frame_id='body';expected.header.stamp.sec,expected.header.stamp.nanosec=divmod(ns,10**9)
  for obj,axes,values in [(expected.orientation,'xyzw',ori),(expected.angular_velocity,'xyz',g),(expected.linear_acceleration,'xyz',acc)]:
   for axis,value in zip(axes,values):setattr(obj,axis,value)
  self.assertEqual(actual,expected)
 def test_legacy_and_raw_buffers_are_independent(self):
  raw=self.codec.encode(10**9,(1.,2.,3.,4.),(5.,6.,7.),(8.,9.,10.));legacy=self.codec.encode(10**9,(0.,0.,0.,1.),(5.,6.,7.),(0.,0.,0.))
  self.assertEqual(deserialize_message(raw,Imu).linear_acceleration.x,8.)
  self.assertEqual(deserialize_message(legacy,Imu).linear_acceleration.x,0.)
 def test_unverified_layout_uses_ros_decoder(self):
  m=self.message('body');self.codec.template=None;self.assertEqual(self.codec.read(serialize_message(m)),self.codec.read(m));self.assertEqual(self.codec.projected,0)
 def test_nonfinite_fields_are_not_hidden_from_existing_validator(self):
  m=self.message('body');m.angular_velocity.x=float('nan');m.orientation.w=float('inf');r=self.codec.read(serialize_message(m));self.assertTrue(math.isnan(r[3][0]));self.assertTrue(math.isinf(r[2][3]))
 def test_invalid_header_shape_is_not_projected(self):
  wire=bytearray(serialize_message(self.message('body')));struct.pack_into('<I',wire,12,0xffffffff)
  self.assertIsNone(self.codec._start(wire));self.assertIsNone(self.codec._start(b''))
 def test_stamp_nanoseconds_preserved_for_invalid_stamp_guard(self):
  m=self.message('body');m.header.stamp.sec=-1;m.header.stamp.nanosec=1000000001
  self.assertEqual(self.codec.read(serialize_message(m))[:2],(-1,1000000001))

if __name__=='__main__':unittest.main()
