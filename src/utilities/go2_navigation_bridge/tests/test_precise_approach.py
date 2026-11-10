"""Desired accuracy and bounded acceptable fallback, no robot or ROS."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_goal import GoalTracker

class PreciseApproach(unittest.TestCase):
    def goal(self):
        g=GoalTracker(.1, .5, acceptable_m=.25, far_mode=True)
        g.accept((0,0,0),'map',1,1.);g.active();return g
    def pose(self,g,x,t,allowed=True,stopped=True):
        g.position(x,0.,t,allowed,stamp_ns=int(t*1e9),measurement_time=t,
                   stopped_time=t if stopped else None)
    def settle(self,g,x,start):
        for i in range(8):self.pose(g,x,start+i*.1)
    def test_original_trial_241mm_does_not_stop_approach(self):
        g=self.goal();self.settle(g,.2413123,2.)
        self.assertEqual(g.state,'ACTIVE');self.assertFalse(g.paused)
    def test_desired_accuracy_still_needs_fresh_stable_stop(self):
        g=self.goal();self.pose(g,.09,2.,stopped=False)
        self.assertEqual(g.state,'ARRIVAL_CONFIRMING')
        self.settle(g,.09,2.1)
        self.assertEqual(g.state,'SUCCEEDED')
        self.assertEqual(g.snapshot(3.)['completion_quality'],'DESIRED')
    def test_acceptable_is_only_after_budget_and_new_confirmation(self):
        g=self.goal();self.pose(g,.24,2.);g.state='REPLANNING';g.wait_updated=2.
        g.advance(11.9);self.assertEqual(g.state,'REPLANNING')
        g.advance(12.01);self.assertEqual(g.state,'ARRIVAL_CONFIRMING')
        self.assertTrue(g.limited_approach)
        self.pose(g,.24,12.1,allowed=False);self.assertIsNone(g.inside_since)
        self.settle(g,.24,12.2)
        self.assertEqual(g.state,'SUCCEEDED')
        self.assertEqual(g.snapshot(13.)['completion_quality'],'ACCEPTABLE')
    def test_fallback_cannot_accept_outside_radius_or_stale_stop(self):
        for x in [.24,.26]:
            g=self.goal();self.pose(g,x,2.);g.state='REPLANNING';g.wait_updated=2.
            g.advance(12.01)
            if x>.25:self.assertEqual(g.state,'CONTINUATION_NOT_FOUND');continue
            self.pose(g,.26,12.1);self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')
        g=self.goal();self.pose(g,.24,2.);g.continuation_exhausted('GOAL_UNREACHABLE',3.)
        g.advance(13.01);self.assertEqual(g.state,'ARRIVAL_NOT_CONFIRMED')
    def test_faults_never_become_acceptable_success(self):
        for reason in ['AUTHORITY_LOST','LOCALIZATION_LOST','CANCELLED']:
            g=self.goal();self.pose(g,.24,2.);g.finish(reason,3.)
            self.settle(g,.09,4.);self.assertEqual(g.state,reason)

    def test_revised_duplicate_outside_fallback_cannot_resume_motion(self):
        g=self.goal();self.pose(g,.24,2.)
        g.continuation_exhausted('CONTINUATION_NOT_FOUND',3.)
        self.pose(g,.26,2.)
        self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')

    def test_new_goal_does_not_inherit_acceptable_fallback(self):
        g=self.goal();self.pose(g,.24,2.)
        g.continuation_exhausted('CONTINUATION_NOT_FOUND',3.)
        self.settle(g,.24,3.1)
        g.accept((1.,0.,0.),'map',2,5.)
        self.assertFalse(g.limited_approach)
        self.assertEqual(g.completion_tolerance,.1)

if __name__=='__main__':unittest.main()
