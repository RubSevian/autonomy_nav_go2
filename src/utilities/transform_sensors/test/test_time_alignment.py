"""Clock invariants with delayed scans, outages and discontinuities; no ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from transform_sensors.time_alignment import AlignmentConfig, SensorTimeAlignment

NS = 10**9
SOURCE = 1730721639 * NS
ROS = 1791732363 * NS
MONO = 1000 * NS


class TimeAlignmentTest(unittest.TestCase):
    def make_alignment(self):
        return SensorTimeAlignment(AlignmentConfig(warmup_s=.1, min_cloud_samples=2, min_imu_samples=2))

    def send(self, clock, stream, seconds, delay=.004, span=0.):
        stamp = SOURCE + round(seconds * NS)
        arrival = round((seconds + delay) * NS)
        return clock.map_stamp(stream, stamp, ROS + arrival, MONO + arrival, round(span * NS))

    def locked(self):
        clock = self.make_alignment()
        self.assertIsNone(self.send(clock, 'cloud', 0., delay=.15, span=.06))
        self.assertIsNone(self.send(clock, 'imu', .15))
        self.assertIsNone(self.send(clock, 'cloud', .1, delay=.07, span=.06))
        self.assertIsNotNone(self.send(clock, 'imu', .255))
        self.assertEqual(clock.state, 'LOCKED')
        return clock

    def test_startup_uses_both_streams_and_not_first_delayed_cloud(self):
        clock = self.locked()
        self.assertEqual(clock.offset_ns, ROS - SOURCE + round((.004 - .02) * NS))
        mapped = self.send(clock, 'cloud', .3, delay=.07, span=.06)
        self.assertLessEqual(mapped + round(.06 * NS), ROS + round(.37 * NS))

    def test_same_offset_preserves_cross_stream_intervals(self):
        clock = self.locked()
        imu = self.send(clock, 'imu', .3)
        cloud = self.send(clock, 'cloud', .28, delay=.07, span=.06)
        self.assertEqual(imu - cloud, round(.02 * NS))
        offset = clock.offset_ns
        later = self.send(clock, 'imu', .36)
        self.assertEqual(later - imu, round(.06 * NS))
        self.assertEqual(clock.offset_ns, offset)

    def test_one_stream_never_starts_output(self):
        clock = self.make_alignment()
        for i in range(200):
            self.assertIsNone(self.send(clock, 'imu', i * .004))
        self.assertEqual(clock.state, 'WARMUP')

    def test_stale_packet_is_not_restamped_as_fresh(self):
        clock = self.locked()
        offset = clock.offset_ns
        self.assertIsNone(self.send(clock, 'imu', .3, delay=.6))
        self.assertEqual(clock.reason, 'measurement_stale')
        self.assertEqual(clock.state, 'LOCKED')
        self.assertIsNotNone(self.send(clock, 'imu', 1.))
        self.assertEqual(clock.offset_ns, offset)

    def test_duplicate_and_small_reordering_do_not_move_clock(self):
        clock = self.locked()
        self.assertIsNone(self.send(clock, 'imu', .255, delay=.01))
        self.assertIsNone(self.send(clock, 'imu', .254, delay=.02))
        self.assertEqual(clock.reason, 'duplicate_or_out_of_order')
        self.assertEqual(clock.state, 'LOCKED')
        self.assertIsNotNone(self.send(clock, 'imu', .3))

    def test_future_sample_faults_both_streams_without_clamping(self):
        clock = self.locked()
        offset = clock.offset_ns
        self.assertIsNone(clock.map_stamp('imu', SOURCE + round(.3 * NS),
                                         ROS + round(.27 * NS), MONO + round(.3 * NS)))
        self.assertEqual(clock.reason, 'measurement_in_future')
        self.assertIsNone(self.send(clock, 'cloud', .4, delay=.07, span=.06))
        self.assertEqual(clock.state, 'FAULT')
        self.assertEqual(clock.offset_ns, offset)

    def test_host_clock_steps_require_restart(self):
        for step in (-NS, NS):
            clock = self.locked()
            self.assertIsNone(clock.map_stamp('imu', SOURCE + round(.3 * NS),
                                             ROS + round(.304 * NS) + step, MONO + round(.304 * NS)))
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
        self.assertEqual(clock.seen, dict(imu=0, cloud=0))
        self.assertIsNone(clock.offset_ns)

    def test_configuration_rejects_nan_boolean_and_inconsistent_deadlines(self):
        for kw in (dict(margin_s=float('nan')), dict(warmup_s=0), dict(min_imu_samples=True),
                   dict(min_cloud_samples=1.5), dict(max_age_s=.1)):
            with self.assertRaises(ValueError):
                AlignmentConfig(**kw)

    def test_many_samples_keep_fixed_offset_and_bounded_state(self):
        clock = self.locked()
        offset = clock.offset_ns
        for i in range(5000):
            self.assertIsNotNone(self.send(clock, 'imu', .3 + i * .004))
        self.assertEqual(clock.offset_ns, offset)
        self.assertEqual(len(clock._last_source), 2)
        self.assertEqual(set(clock.status()['seen']), {'cloud', 'imu'})


if __name__ == '__main__':
    unittest.main()
