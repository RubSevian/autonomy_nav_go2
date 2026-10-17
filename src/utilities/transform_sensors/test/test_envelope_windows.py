"""Window-boundary regressions: pure timestamp fixtures, no ROS transport."""
import unittest
from test_bounded_alignment import BoundedTimeAlignment,events,S,R,M,NS


def boundary_rows():
    rows=events(3)
    for i in range(301):
        t=3*NS+i*4_000_000
        rows.append((t+2_000_000,'imu',S+t,0))
    for i in range(7):
        # The cloud opens the [3.002,4.002) window; an IMU closes it.
        t=3*NS+i*200_000_000+(0 if i<5 else 1_000_000)
        rows.append((t+2_000_000,'cloud',S+t,0))
    return sorted(rows)


class EnvelopeWindowTests(unittest.TestCase):
    def test_valid_five_cloud_window_keeps_lock_when_closing_stream_changes(self):
        clock=BoundedTimeAlignment()
        for arrival,stream,source,span in boundary_rows():
            clock.map_stamp(stream,source,R+arrival,M+arrival,span)
        self.assertEqual(clock.state,'LOCKED',clock.status())
        windows=clock.status()['envelope_window_history']
        closed=next(w for w in windows if w['start_mono_ns']==M+3*NS+2_000_000)
        self.assertEqual(closed['counts'],dict(cloud=5,imu=250))
        self.assertEqual(closed['closing_stream'],'imu')
        self.assertTrue(closed['sufficient'])

    def test_real_four_cloud_window_still_latches_degraded(self):
        clock=BoundedTimeAlignment()
        for arrival,stream,source,span in boundary_rows():
            if stream=='cloud' and source==S+3*NS+400_000_000:
                continue
            clock.map_stamp(stream,source,R+arrival,M+arrival,span)
            if clock.state=='DEGRADED':break
        self.assertEqual(clock.failure_reason,'insufficient_common_envelope_samples')
        failed=clock.status()['envelope_window_history'][-1]
        self.assertEqual(failed['counts']['cloud'],4)
        self.assertEqual(failed['required_counts'],dict(cloud=5,imu=100))
        self.assertFalse(failed['sufficient'])

    def test_closing_cloud_cannot_rescue_underfilled_old_window(self):
        clock=BoundedTimeAlignment()
        rows=[]
        for arrival,stream,source,span in boundary_rows():
            # Start window with IMU instead of cloud. Only four cloud samples
            # are inside it; the fifth arrives at its right boundary.
            if stream=='cloud' and source==S+3*NS:continue
            if stream=='cloud' and source==S+4*NS+1_000_000:
                arrival-=1_000_000;source-=1_000_000
            rows.append((arrival,stream,source,span))
        for arrival,stream,source,span in sorted(rows):
            clock.map_stamp(stream,source,R+arrival,M+arrival,span)
            if clock.state=='DEGRADED':break
        self.assertEqual(clock.failure_reason,'insufficient_common_envelope_samples')
        failed=clock.status()['envelope_window_history'][-1]
        self.assertEqual(failed['counts']['cloud'],4)
        self.assertEqual(failed['closing_stream'],'cloud')

    def test_window_history_is_bounded(self):
        clock=BoundedTimeAlignment()
        for arrival,stream,source,span in events(30):
            clock.map_stamp(stream,source,R+arrival,M+arrival,span)
        self.assertEqual(clock.state,'LOCKED')
        self.assertEqual(len(clock.status()['envelope_window_history']),8)


if __name__=='__main__':unittest.main()
