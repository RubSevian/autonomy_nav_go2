# Copyright (c) 2026 RubSevian
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Clock invariants with delayed scans, outages and discontinuities; no ROS."""

from pathlib import Path
import sys
import unittest

from transform_sensors.time_alignment import AlignmentConfig
from transform_sensors.time_alignment import SensorTimeAlignment

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

NS = 10**9
SOURCE = 1730721639 * NS
ROS = 1791732363 * NS
MONO = 1000 * NS


class TimeAlignmentTest(unittest.TestCase):
    def make_alignment(self):
        return SensorTimeAlignment(
            AlignmentConfig(warmup_s=0.1, min_cloud_samples=2, min_imu_samples=2)
        )

    def send(self, clock, stream, seconds, delay=0.004, span=0.0):
        stamp = SOURCE + round(seconds * NS)
        arrival = round((seconds + delay) * NS)
        return clock.map_stamp(stream, stamp, ROS + arrival, MONO + arrival, round(span * NS))

    def locked(self):
        clock = self.make_alignment()
        self.assertIsNone(self.send(clock, 'cloud', 0.0, delay=0.15, span=0.06))
        self.assertIsNone(self.send(clock, 'imu', 0.15))
        self.assertIsNone(self.send(clock, 'cloud', 0.1, delay=0.07, span=0.06))
        self.assertIsNotNone(self.send(clock, 'imu', 0.255))
        self.assertEqual(clock.state, 'LOCKED')
        return clock

    def test_startup_uses_both_streams_and_not_first_delayed_cloud(self):
        clock = self.locked()
        self.assertEqual(clock.offset_ns, ROS - SOURCE + round((0.004 - 0.02) * NS))
        mapped = self.send(clock, 'cloud', 0.3, delay=0.07, span=0.06)
        self.assertLessEqual(mapped + round(0.06 * NS), ROS + round(0.37 * NS))

    def test_same_offset_preserves_cross_stream_intervals(self):
        clock = self.locked()
        imu = self.send(clock, 'imu', 0.3)
        cloud = self.send(clock, 'cloud', 0.28, delay=0.07, span=0.06)
        self.assertEqual(imu - cloud, round(0.02 * NS))
        offset = clock.offset_ns
        later = self.send(clock, 'imu', 0.36)
        self.assertEqual(later - imu, round(0.06 * NS))
        self.assertEqual(clock.offset_ns, offset)

    def test_one_stream_never_starts_output(self):
        clock = self.make_alignment()
        for i in range(200):
            self.assertIsNone(self.send(clock, 'imu', i * 0.004))
        self.assertEqual(clock.state, 'WARMUP')

    def test_stale_packet_is_not_restamped_as_fresh(self):
        clock = self.locked()
        offset = clock.offset_ns
        self.assertIsNone(self.send(clock, 'imu', 0.3, delay=0.6))
        self.assertEqual(clock.reason, 'measurement_stale')
        self.assertEqual(clock.state, 'LOCKED')
        self.assertIsNotNone(self.send(clock, 'imu', 1.0))
        self.assertEqual(clock.offset_ns, offset)

    def test_duplicate_and_small_reordering_do_not_move_clock(self):
        clock = self.locked()
        self.assertIsNone(self.send(clock, 'imu', 0.255, delay=0.01))
        self.assertIsNone(self.send(clock, 'imu', 0.254, delay=0.02))
        self.assertEqual(clock.reason, 'duplicate_or_out_of_order')
        self.assertEqual(clock.state, 'LOCKED')
        self.assertIsNotNone(self.send(clock, 'imu', 0.3))

    def test_future_sample_faults_both_streams_without_clamping(self):
        clock = self.locked()
        offset = clock.offset_ns
        self.assertIsNone(
            clock.map_stamp(
                'imu', SOURCE + round(0.3 * NS), ROS + round(0.27 * NS), MONO + round(0.3 * NS)
            )
        )
        self.assertEqual(clock.reason, 'measurement_in_future')
        self.assertIsNone(self.send(clock, 'cloud', 0.4, delay=0.07, span=0.06))
        self.assertEqual(clock.state, 'FAULT')
        self.assertEqual(clock.offset_ns, offset)

    def test_host_clock_steps_require_restart(self):
        for step in (-NS, NS):
            clock = self.locked()
            self.assertIsNone(
                clock.map_stamp(
                    'imu',
                    SOURCE + round(0.3 * NS),
                    ROS + round(0.304 * NS) + step,
                    MONO + round(0.304 * NS),
                )
            )
            self.assertEqual(clock.state, 'FAULT')
            self.assertEqual(clock.reason, 'host_clock_jump')

    def test_sensor_restart_stops_session(self):
        clock = self.locked()
        self.assertIsNone(clock.map_stamp('imu', SOURCE - NS, ROS + NS, MONO + NS))
        self.assertEqual(clock.state, 'FAULT')
        self.assertEqual(clock.reason, 'sensor_clock_reset')

    def test_invalid_stamps_and_scan_spans_do_not_seed_alignment(self):
        clock = self.make_alignment()
        for stamp, span in ((0, 0), (-1, 0), (SOURCE, -1), (SOURCE, NS)):
            self.assertIsNone(clock.map_stamp('cloud', stamp, ROS, MONO, span))
        self.assertEqual(clock.seen, {'imu': 0, 'cloud': 0})
        self.assertIsNone(clock.offset_ns)

    def test_configuration_rejects_nan_boolean_and_inconsistent_deadlines(self):
        for kw in (
            {'margin_s': float('nan')},
            {'warmup_s': 0},
            {'min_imu_samples': True},
            {'min_cloud_samples': 1.5},
            {'max_age_s': 0.1},
        ):
            with self.assertRaises(ValueError):
                AlignmentConfig(**kw)

    def test_many_samples_keep_fixed_offset_and_bounded_state(self):
        clock = self.locked()
        offset = clock.offset_ns
        for i in range(5000):
            self.assertIsNotNone(self.send(clock, 'imu', 0.3 + i * 0.004))
        self.assertEqual(clock.offset_ns, offset)
        self.assertEqual(len(clock._last_source), 2)
        self.assertEqual(set(clock.status()['seen']), {'cloud', 'imu'})


if __name__ == '__main__':
    unittest.main()
