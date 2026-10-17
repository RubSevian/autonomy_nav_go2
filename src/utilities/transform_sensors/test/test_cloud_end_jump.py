import unittest
from test_bounded_alignment import BoundedTimeAlignment, locked, S, R, M, NS

class CloudEndTests(unittest.TestCase):
 def test_real_cloud_forward_jump_still_faults(self):
  c=locked();self.assertIsNone(c.map_stamp('cloud',S+5*NS,R+3*NS,M+3*NS,64_000_000));self.assertEqual(c.failure_reason,'sensor_clock_forward_jump')
 def test_cloud_clock_jump_not_hidden_by_shorter_span(self):
  c=locked();self.assertIsNone(c.map_stamp('cloud',S+5*NS,R+3*NS,M+3*NS,0));self.assertEqual(c.state,'FAULT')
 def test_cloud_source_reset_still_faults(self):
  c=locked();self.assertIsNone(c.map_stamp('cloud',S-NS,R+3*NS,M+3*NS,64_000_000));self.assertEqual(c.failure_reason,'sensor_clock_reset')
 def test_cloud_start_order_check_remains(self):
  c=locked();source=c._last_source['cloud'];self.assertIsNone(c.map_stamp('cloud',source,R+3*NS,M+3*NS,190_000_000));self.assertEqual(c.reason,'duplicate_or_out_of_order');self.assertEqual(c.state,'LOCKED')
if __name__ == '__main__':unittest.main()
