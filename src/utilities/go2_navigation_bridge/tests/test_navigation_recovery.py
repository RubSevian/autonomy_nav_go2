"""Pure recovery tests; no ROS init or velocity publication."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_recovery import LocalizationRecovery, PathRecovery
from navigation_gate import NavigationGate, ZERO
from navigation_goal import GoalTracker


class RecoveryTests(unittest.TestCase):
    def test_five_new_samples_no_duplicate_renewal(self):
        r=LocalizationRecovery()
        for n in range(1,5):self.assertFalse(r.observe(n,True))
        self.assertFalse(r.observe(4,True));self.assertEqual(r.good,4)
        self.assertTrue(r.observe(5,True));r.invalidate()
        self.assertFalse(r.observe(5,True));self.assertEqual(r.good,0)
        for n in range(6,10):self.assertFalse(r.observe(n,True))
        self.assertTrue(r.observe(10,True))

    def test_invalid_or_reversed_stamp_breaks_consecutive_sequence(self):
        r=LocalizationRecovery()
        for n in range(1,5):r.observe(n,True)
        self.assertFalse(r.observe(5,False));self.assertEqual(r.good,0)
        for n in range(6,10):self.assertFalse(r.observe(n,True))
        self.assertFalse(r.observe(8,True));self.assertEqual(r.good,0)

    def test_odom_499_501ms_and_goal_invalidation(self):
        g=NavigationGate(recovery_samples=5)
        for name in ('lidar','imu','odom','tf','path','stop'):g.observe(name,True,100.)
        for n in range(1,6):g.observe_pair(n,True)
        g.locomotion(True,100.);g.planner_active(True,100.);g.goal()
        self.assertTrue(g.localization_ready(100.499))
        self.assertFalse(g.localization_ready(100.501));self.assertTrue(g.goal_invalidated)
        self.assertFalse(g.goal_received);self.assertEqual(g.output(100.501),ZERO)
        for n in range(6,11):
            now=100.6+n*.01
            for name in ('lidar','imu','odom','tf'):g.observe(name,True,now)
            g.observe_pair(n,True)
        self.assertTrue(g.localization_ready(now));self.assertFalse(g.goal_received)
        self.assertFalse(g.enabled(now)) # Five samples cannot resurrect a goal/path.

    def test_soft_goal_invalidation_never_clears_explicit_cancel(self):
        for cancelled in (False,True):
            g=NavigationGate(recovery_samples=5)
            if cancelled:g.cancel()
            g.goal_received=True;g.observe('path',True,100.)
            g.invalidate_goal()
            self.assertEqual(g.cancelled,cancelled)
            self.assertFalse(g.goal_received);self.assertIsNone(g.health['path'])
            self.assertEqual(g.output(100.),ZERO)

    def check(self,p,now,**changes):
        args=dict(goal_key=('nav',3,100),localization_valid=True,authority_valid=True,
                  path_valid=False,path_source_time=None);args.update(changes)
        return p.check(now,**args)

    def test_path_recovers_same_goal_before_ten_seconds(self):
        p=PathRecovery()
        self.assertEqual(self.check(p,100.),'WAITING_FOR_PATH')
        self.assertEqual(self.check(p,109.9,path_valid=True,path_source_time=99.),'WAITING_FOR_PATH')
        self.assertEqual(self.check(p,109.99,path_valid=True,path_source_time=109.),'RECOVERED')
        self.assertIsNone(p.since)

    def test_deadline_or_authority_changes_prevent_resume(self):
        for changes in ({'localization_valid':False},{'authority_valid':False},{'goal_key':('nav2',3,100)}):
            p=PathRecovery();self.check(p,100.)
            self.assertEqual(self.check(p,101.,**changes),'INVALIDATE_GOAL')
        p=PathRecovery();self.check(p,100.)
        self.assertEqual(self.check(p,110.,path_valid=True,path_source_time=110.),'GOAL_UNREACHABLE')

    def test_operator_can_reissue_same_coordinates_only_with_new_stamp(self):
        g=GoalTracker();self.assertTrue(g.accept((1,2,0),'map',100,1.))
        g.finish('LOCALIZATION_LOST',2.)
        self.assertFalse(g.accept((1,2,0),'map',100,3.))
        self.assertTrue(g.accept((1,2,0),'map',200,3.))
        self.assertEqual(g.revision,2)


if __name__=='__main__':unittest.main()
