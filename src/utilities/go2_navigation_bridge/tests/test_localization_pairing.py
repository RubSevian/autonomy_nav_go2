import json
from pathlib import Path
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from localization_pairing import (LocalizationPairing, PairingConfig, PoseSample,
                                  PairLookupError, pose_errors)
from navigation_gate import NavigationGate, ZERO


def sample(stamp=10_000_000_000, x=0., quaternion=(0., 0., 0., 1.)):
    return PoseSample(stamp, 'camera_init', 'aft_mapped', (x, 0., 0.), quaternion)


def vehicle(odom, tf):
    return replace(tf, frame='map', child='vehicle',
                   position=(tf.position[0] - .3, *tf.position[1:]))


class PairingTest(unittest.TestCase):
    def test_four_actual_messages_and_equal_stamp_revisions(self):
        rows = json.loads((Path(__file__).parent / 'data/tf_duplicate_revisions.json').read_text())['messages']
        pairing = LocalizationPairing(PairingConfig())
        observed = []
        for row in rows:
            pose = PoseSample(row['stamp'], 'camera_init', 'aft_mapped',
                              tuple(row['pose'][:3]), tuple(row['pose'][3:]))
            (pairing.observe_odom if row['kind'] == 'odom' else pairing.observe_tf)(pose)
            observed.append(pairing.select(row['stamp'] + int(row['age_s']*1e9), vehicle))
        self.assertIsNone(observed[0])
        self.assertIsNone(observed[1])
        self.assertEqual(observed[2].odom.position, tuple(rows[0]['pose'][:3]))
        self.assertEqual(observed[3].odom.position, tuple(rows[1]['pose'][:3]))
        self.assertEqual(observed[3].position_error_m, 0.)
        self.assertEqual(pairing.odom.count, 2)

    def test_odom_before_tf_uses_previous_without_changing_measurement_time(self):
        p = LocalizationPairing(PairingConfig())
        first, second = sample(), sample(10_020_000_000, .1)
        p.observe_odom(first); p.observe_tf(first)
        pair = p.select(10_050_000_000, vehicle)
        p.observe_odom(second)
        old = p.select(10_072_000_000, vehicle)
        self.assertEqual(old.odom.stamp_ns, pair.odom.stamp_ns)
        self.assertTrue(p.fallback)
        self.assertAlmostEqual(p.snapshot(10_072_000_000)['selected_age_s'], .072)
        p.observe_tf(second)
        new = p.select(10_073_000_000, vehicle)
        self.assertEqual(new.odom, second)
        self.assertFalse(p.fallback)
        self.assertEqual(new.vehicle.stamp_ns, second.stamp_ns)

    def test_tf_before_odom_and_same_stamp_revision_fallback(self):
        p = LocalizationPairing(PairingConfig())
        first, revised = sample(), sample(x=.002)
        p.observe_odom(first); p.observe_tf(first)
        selected = p.select(10_050_000_000, vehicle)
        p.observe_tf(revised)
        self.assertIs(p.select(10_070_000_000, vehicle), selected)
        self.assertEqual(p.waiting_reason, 'pair_pose_mismatch')
        self.assertTrue(p.fallback)
        p.observe_odom(revised)
        self.assertEqual(p.select(10_080_000_000, vehicle).odom, revised)
        self.assertFalse(p.fallback)

    def test_genuine_mismatch_blocks_and_quaternion_sign_is_equivalent(self):
        p = LocalizationPairing(PairingConfig())
        p.observe_odom(sample()); p.observe_tf(sample(x=.002))
        self.assertIsNone(p.select(10_050_000_000, vehicle))

        self.assertEqual(p.reason, 'pair_pose_mismatch')
        p.observe_tf(sample(quaternion=(0., 0., 0., -1.)))
        self.assertEqual(p.select(10_060_000_000, vehicle).quaternion_error, 0.)
        p.clear(); p.observe_odom(sample())
        p.observe_tf(sample(quaternion=(0., 0., .02, 1.)))
        self.assertIsNone(p.select(10_050_000_000, vehicle))

    def test_small_numeric_differences_use_configured_tolerances(self):
        for delta, tolerance, accepted in ((.0007, .001, True), (.0015, .001, False),
                                            (.0015, .002, True)):
            p = LocalizationPairing(PairingConfig(position_tolerance_m=tolerance))
            p.observe_odom(sample())
            p.observe_tf(sample(x=delta, quaternion=(0., 0., .0005, 1.)))
            self.assertEqual(p.select(10_050_000_000, vehicle) is not None, accepted)

    def test_both_streams_lost_expires_and_revokes_nonzero_nav(self):
        p = LocalizationPairing(PairingConfig())
        p.observe_odom(sample()); p.observe_tf(sample())
        self.assertIsNotNone(p.select(10_050_000_000, vehicle))
        for tick in (10_100_000_000, 10_300_000_000, 10_500_000_000):
            self.assertEqual(p.select(tick, vehicle).odom.stamp_ns, 10_000_000_000)
        g = NavigationGate('stack')
        g.goal(); g.locomotion(True, 20.); g.planner_active(True, 20.)
        for key in ('imu', 'lidar', 'odom', 'tf', 'path', 'stop'):
            g.observe(key, True, 20.)
        self.assertTrue(g.accept((.1, 0., 0.), 0., 'vehicle', 20.))
        self.assertIsNone(p.select(10_500_000_001, vehicle))
        self.assertEqual(p.reason, 'pair_expired')
        g.observe('tf', False, 20.)
        self.assertFalse(g.enabled(20.))
        self.assertEqual(g.output(20.), ZERO)

    def test_one_stream_lost_new_messages_do_not_renew_cached_pair(self):
        for stream in ('odom', 'tf'):
            p = LocalizationPairing(PairingConfig())
            p.observe_odom(sample()); p.observe_tf(sample())
            p.select(10_050_000_000, vehicle)
            for ns in range(10_100_000_000, 10_700_000_000, 100_000_000):
                getattr(p, 'observe_' + stream)(sample(ns))
                result = p.select(ns, vehicle)
                if ns <= 10_500_000_000:
                    self.assertEqual(result.odom.stamp_ns, 10_000_000_000)
                else:
                    self.assertIsNone(result)

    def test_close_but_different_stamps_are_never_a_pair(self):
        p = LocalizationPairing(PairingConfig())
        p.observe_odom(sample()); p.observe_tf(sample(10_000_000_001))
        self.assertIsNone(p.select(10_050_000_000, vehicle))
        self.assertEqual(p.reason, 'no_common_timestamp')

    def test_future_invalid_and_clock_regression(self):
        p = LocalizationPairing(PairingConfig())
        p.observe_odom(sample()); p.observe_tf(sample())
        self.assertIsNone(p.select(9_999_999_999, vehicle))
        self.assertIsNotNone(p.select(10_050_000_000, vehicle))
        self.assertIsNone(p.select(10_040_000_000, vehicle))
        self.assertEqual(p.reason, 'ros_clock_regressed')
        self.assertEqual(p.odom.count, 0)
        p.observe_odom(replace(sample(), position=(float('nan'), 0., 0.)))
        self.assertEqual(p.reason, 'invalid_odometry')

    def test_same_time_chain_required_and_failed_lookup_can_fallback(self):
        p = LocalizationPairing(PairingConfig())
        p.observe_odom(sample()); p.observe_tf(sample())
        self.assertIsNone(p.select(10_050_000_000,
                                  lambda o, t: replace(vehicle(o, t), stamp_ns=o.stamp_ns+1)))
        self.assertEqual(p.reason, 'invalid_vehicle_transform')
        good = p.select(10_060_000_000, vehicle)
        def unavailable(o, t):
            raise PairLookupError('tf_chain_unavailable', 'missing sensor->vehicle')
        self.assertIs(p.select(10_070_000_000, unavailable), good)
        self.assertTrue(p.fallback)
        self.assertEqual(p.waiting_reason, 'tf_chain_unavailable')
        self.assertIsNone(p.select(10_600_000_000, unavailable))

    def test_memory_cap_revisions_and_time_horizon_out_of_order(self):
        p = LocalizationPairing(PairingConfig(max_samples=64))
        for i in range(5000):
            p.observe_odom(sample(10_000_000_000+i*1_000_000))
            p.observe_tf(sample(10_000_000_000+i*1_000_000))
        self.assertEqual(p.odom.count, 64)
        self.assertEqual(p.tf.count, 64)
        p.observe_odom(sample(1_000_000_000))
        self.assertNotIn(1_000_000_000, p.odom.buckets)
        for _ in range(100):
            p.observe_odom(sample(15_000_000_000))
        self.assertEqual(p.odom.count, 64)
        self.assertEqual(len(p.odom.buckets[15_000_000_000]), 64)
        p.select(17_000_000_000, vehicle)
        self.assertEqual(p.odom.count, 0)
        self.assertEqual(p.tf.count, 0)

    def test_time_window_covers_more_than_old_32_samples(self):
        p = LocalizationPairing(PairingConfig())
        first = sample()
        p.observe_odom(first); p.observe_tf(first)
        for i in range(1, 200):
            p.observe_odom(sample(first.stamp_ns+i*1_000_000))
        self.assertEqual(p.select(10_300_000_000, vehicle).odom, first)

    def test_late_packets_never_regress_selected_time(self):
        p = LocalizationPairing(PairingConfig())
        newest = sample(10_100_000_000, .1)
        p.observe_odom(newest); p.observe_tf(newest)
        p.select(10_150_000_000, vehicle)
        p.observe_odom(sample()); p.observe_tf(sample())
        self.assertEqual(p.select(10_160_000_000, vehicle).odom, newest)

    def test_same_stamp_revision_cannot_mask_cached_fallback_with_older_pair(self):
        p = LocalizationPairing(PairingConfig())
        old, newest = sample(), sample(10_100_000_000, .1)
        for pose in (old, newest):
            p.observe_odom(pose); p.observe_tf(pose)
        good = p.select(10_150_000_000, vehicle)
        p.observe_tf(replace(newest, position=(.2, 0., 0.)))
        self.assertIs(p.select(10_160_000_000, vehicle), good)
        self.assertTrue(p.fallback)
        self.assertEqual(p.waiting_reason, 'pair_pose_mismatch')

    def test_config_validation(self):
        for kwargs in (dict(history_s=.4), dict(max_samples=True), dict(max_samples=1.5),
                       dict(max_age_s=0), dict(position_tolerance_m=float('nan')),
                       dict(quaternion_tolerance=-1)):
            with self.assertRaises(ValueError):
                PairingConfig(**kwargs)


if __name__ == '__main__':
    unittest.main()
