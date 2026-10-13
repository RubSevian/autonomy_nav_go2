"""Native Jazzy TF buffer regression, with no ROS node, DDS or publishers."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from go2_navigation_bridge import LocalizationBuffer, NavigationBridge
from localization_pairing import LocalizationPairing, PairingConfig, PoseSample


def transform(pose):
    msg = TransformStamped()
    msg.header.stamp = Time(nanoseconds=pose.stamp_ns).to_msg()
    msg.header.frame_id, msg.child_frame_id = pose.frame, pose.child
    p, q = msg.transform.translation, msg.transform.rotation
    p.x, p.y, p.z = pose.position
    q.x, q.y, q.z, q.w = pose.quaternion
    return msg


class BufferTest(unittest.TestCase):
    def setup_pairing(self):
        pairing = LocalizationPairing(PairingConfig())
        buffer = LocalizationBuffer(pairing)
        for frame, child, position in (('map', 'camera_init', (0., 0., 0.)),
                                        ('aft_mapped', 'sensor', (0., 0., 0.)),
                                        ('sensor', 'vehicle', (-.3, 0., 0.))):
            buffer.set_transform_static(transform(PoseSample(0, frame, child, position,
                                                             (0., 0., 0., 1.))), 'test')
        state = SimpleNamespace(tf_buffer=buffer, pairing=pairing)
        return pairing, buffer, lambda o, t: NavigationBridge.resolve_vehicle(state, o, t)

    def test_captured_revisions_through_actual_buffer(self):
        pairing, buffer, resolve = self.setup_pairing()
        rows = json.loads((Path(__file__).parent / 'data/tf_duplicate_revisions.json').read_text())['messages']
        selected = []
        for row in rows:
            pose = PoseSample(row['stamp'], 'camera_init', 'aft_mapped',
                              tuple(row['pose'][:3]), tuple(row['pose'][3:]))
            if row['kind'] == 'odom':
                pairing.observe_odom(pose)
            else:
                buffer.set_transform(transform(pose), 'test')
            selected.append(pairing.select(row['stamp']+int(row['age_s']*1e9), resolve))
        self.assertIsNone(selected[1])
        self.assertEqual(selected[2].odom.position, tuple(rows[0]['pose'][:3]))
        self.assertEqual(selected[3].odom.position, tuple(rows[1]['pose'][:3]))
        self.assertEqual(selected[3].vehicle.stamp_ns, rows[1]['stamp'])
        self.assertEqual(selected[3].vehicle.position,
                         tuple(getattr(buffer.lookup_transform('map', 'vehicle',
                               Time(nanoseconds=rows[1]['stamp'])).transform.translation, axis)
                               for axis in ('x', 'y', 'z')))

    def test_new_tf_before_odom_retains_verified_vehicle_pose(self):
        pairing, buffer, resolve = self.setup_pairing()
        pose = PoseSample(10_000_000_000, 'camera_init', 'aft_mapped',
                          (1., 0., 0.), (0., 0., 0., 1.))
        pairing.observe_odom(pose)
        buffer.set_transform(transform(pose), 'test')
        first = pairing.select(10_050_000_000, resolve)
        revised = PoseSample(pose.stamp_ns, pose.frame, pose.child, (2., 0., 0.), pose.quaternion)
        buffer.set_transform(transform(revised), 'test')
        self.assertIs(pairing.select(10_070_000_000, resolve), first)
        self.assertEqual(first.vehicle.position, (.7, 0., 0.))
        pairing.observe_odom(revised)
        self.assertEqual(pairing.select(10_080_000_000, resolve).vehicle.position, (1.7, 0., 0.))

    def test_whole_chain_uses_selected_stamp_and_never_latest(self):
        pairing, buffer, resolve = self.setup_pairing()
        old = PoseSample(10_000_000_000, 'camera_init', 'aft_mapped',
                         (1., 0., 0.), (0., 0., 0., 1.))
        newer = PoseSample(10_100_000_000, old.frame, old.child, (2., 0., 0.), old.quaternion)
        pairing.observe_odom(old)
        buffer.set_transform(transform(old), 'test')
        buffer.set_transform(transform(newer), 'test')  # no matching new odom yet
        calls = []
        original = buffer.lookup_transform
        def lookup(target, source, stamp, *args, **kwargs):
            calls.append(stamp.nanoseconds)
            return original(target, source, stamp, *args, **kwargs)
        buffer.lookup_transform = lookup
        pair = pairing.select(10_150_000_000, resolve)
        self.assertEqual(pair.vehicle.position, (.7, 0., 0.))
        self.assertEqual(calls, [old.stamp_ns, old.stamp_ns])
        self.assertEqual(pair.vehicle.stamp_ns, old.stamp_ns)

    def test_missing_static_chain_reports_reason(self):
        pairing = LocalizationPairing(PairingConfig())
        buffer = LocalizationBuffer(pairing)
        pose = PoseSample(10_000_000_000, 'camera_init', 'aft_mapped',
                          (0., 0., 0.), (0., 0., 0., 1.))
        pairing.observe_odom(pose)
        buffer.set_transform(transform(pose), 'test')
        state = SimpleNamespace(tf_buffer=buffer, pairing=pairing)
        self.assertIsNone(pairing.select(10_050_000_000,
                          lambda o, t: NavigationBridge.resolve_vehicle(state, o, t)))
        self.assertEqual(pairing.reason, 'tf_chain_unavailable')
        self.assertTrue(pairing.detail)


if __name__ == '__main__':
    unittest.main()
