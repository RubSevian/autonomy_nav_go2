"""Original-goal outcomes, bounded pauses and stale-message regressions; no ROS."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_goal import GoalTracker
from navigation_contract import locomotion_stopped


class GoalContinuationTests(unittest.TestCase):
    def goal(self, **changes):
        g=GoalTracker(far_mode=True,**changes)
        g.accept((.201860,-.526567,.164641),'map',100,1.)
        return g

    def fields(self,g,reason,**changes):
        f=dict(zip(('goal_original_x','goal_original_y','goal_original_z'),map(str,g.target)))
        f.update(goal_stamp_ns=str(g.stamp_ns),reason_code=reason,
                 goal_current_x='.251860',goal_current_y='-.476567',goal_current_z='.207485')
        f.update(changes);return f

    def status(self,g,reason,stamp,now,**changes):
        return g.far_status(self.fields(g,reason,**changes),stamp,now)

    def position(self,g,x,y,now,stamp,stopped=True,**kw):
        g.position(x,y,now,True,stamp_ns=stamp,measurement_time=now,
                   stopped_time=now if stopped else None,**kw)

    def test_recorded_adjusted_stop_keeps_original_task(self):
        g=self.goal();self.position(g,.096999,-.283226,2.,1000)
        self.status(g,'ADJUSTED_GOAL_WAIT',110,2.)
        self.assertEqual(g.state,'REPLANNING')
        self.assertEqual(g.revision,1);self.assertEqual(g.stamp_ns,100)
        self.assertAlmostEqual(g.error_xy,.264973,places=5)
        self.status(g,'ROUTE_COMMITTED',120,2.5)
        self.assertEqual(g.state,'ACTIVE')
        self.position(g,.12,-.31,3.,2000)
        self.assertEqual(g.state,'ARRIVAL_CONFIRMING')
        self.position(g,.12,-.31,3.2,2100)
        for i in range(1,7):self.position(g,.12,-.31,3.2+i*.1,2100+i)
        self.assertEqual(g.state,'SUCCEEDED')

    def test_far_claim_cannot_complete_goal(self):
        g=self.goal();self.position(g,0.,0.,2.,1000)
        self.status(g,'GOAL_REACHED',110,2.)
        self.assertNotEqual(g.state,'SUCCEEDED')

    def test_duplicate_poses_and_old_zero_do_not_confirm_stop(self):
        g=self.goal();self.position(g,.12,-.31,2.,1000,False)
        g.position(.12,-.31,2.5,True,stamp_ns=1001,stopped_time=1.9)
        self.assertIsNone(g.inside_since)
        self.position(g,.12,-.31,3.,2000)
        self.position(g,.12,-.31,4.,2000)
        self.assertNotEqual(g.state,'SUCCEEDED')
        self.position(g,.12,-.31,4.1,2001)
        self.assertNotEqual(g.state,'SUCCEEDED')
        for i in range(1,7):self.position(g,.12,-.31,4.1+i*.1,2001+i)
        self.assertEqual(g.state,'SUCCEEDED')

    def test_position_or_yaw_motion_restarts_settling(self):
        for yaw,dx in ((.1,0.),(0.,.03)):
            g=self.goal();self.position(g,.12,-.31,2.,1000)
            self.position(g,.12+dx,-.31,2.4,1100,yaw=yaw)
            self.assertNotEqual(g.state,'SUCCEEDED')
            self.position(g,.12+dx,-.31,2.8,1200,yaw=yaw)
            self.assertNotEqual(g.state,'SUCCEEDED')
            for i in range(1,7):self.position(g,.12+dx,-.31,2.8+i*.1,1200+i,yaw=yaw)
            self.assertEqual(g.state,'SUCCEEDED')

    def test_leaving_original_tolerance_restarts_confirmation(self):
        g=self.goal();self.position(g,.12,-.31,2.,1000)
        self.position(g,0.,0.,2.4,1100)
        self.assertEqual(g.state,'REPLANNING')
        self.position(g,.12,-.31,2.5,1200)
        self.position(g,.12,-.31,2.9,1300)
        self.assertNotEqual(g.state,'SUCCEEDED')

    def test_cumulative_budget_cannot_reset_by_route_revisions(self):
        g=self.goal(wait_budget_s=1.)
        self.status(g,'ADJUSTED_GOAL_WAIT',110,2.)
        self.status(g,'ROUTE_COMMITTED',120,2.6)
        self.status(g,'ADJUSTED_GOAL_WAIT',130,10.)
        self.status(g,'ROUTE_COMMITTED',140,10.41)
        self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')
        self.assertEqual(g.revision,1)

    def test_repeated_wait_status_does_not_renew_budget(self):
        g=self.goal(wait_budget_s=1.)
        for i in range(11):self.status(g,'ADJUSTED_GOAL_WAIT',110+i,2.+i*.11)
        self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')

    def test_cancel_and_localization_loss_cannot_be_revived(self):
        for state in ('CANCELLED','LOCALIZATION_LOST','AUTHORITY_LOST','SUCCEEDED'):
            g=self.goal();g.finish(state,2.)
            self.status(g,'ROUTE_COMMITTED',110,2.1)
            self.position(g,.12,-.31,3.,1000)
            self.assertEqual(g.state,state)

    def test_wrong_goal_stamp_and_old_diagnostic_cannot_resume(self):
        g=self.goal();self.status(g,'ADJUSTED_GOAL_WAIT',110,2.)
        self.status(g,'ROUTE_COMMITTED',120,2.1,goal_stamp_ns='99')
        self.status(g,'ROUTE_COMMITTED',109,2.2)
        self.assertEqual(g.state,'REPLANNING')

    def test_new_goal_clears_budget_and_old_reports_are_ignored(self):
        g=self.goal();old=self.fields(g,'ROUTE_COMMITTED')
        self.status(g,'ADJUSTED_GOAL_WAIT',110,2.);g.advance(3.)
        self.assertTrue(g.accept((1.,2.,0.),'map',200,3.))
        self.assertEqual(g.wait_spent,0.)
        self.assertFalse(g.far_status(old,210,3.1));self.assertEqual(g.state,'REQUESTED')

    def test_invalid_or_unready_pose_breaks_confirmation(self):
        g=self.goal();self.position(g,.12,-.31,2.,1000)
        g.position(.12,-.31,2.4,False,stamp_ns=1100)
        self.assertIsNone(g.inside_since)
        self.position(g,.12,-.31,3.,1200)
        self.assertNotEqual(g.state,'SUCCEEDED')

    def test_profile_rejects_inconsistent_wait_and_settle_limits(self):
        import tempfile,yaml
        from navigation_profile import load_profile
        path=Path(__file__).resolve().parents[1]/'config/real_navigation.yaml'
        for key,value in (('goal_wait_budget_s',.5),('goal_settle_position_m',.25),
                          ('goal_settle_yaw_rad',4.),('goal_wait_budget_s',True)):
            profile=load_profile(path);profile['bridge'][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as f:
                yaml.safe_dump(profile,f);f.flush()
                with self.assertRaises(ValueError):load_profile(f.name)

    def test_r3_zero_requires_fresh_valid_report(self):
        import yaml
        d=dict(observed_ns=1_000_000_000,state='ACTIVE',phase='RL_ZERO',command_source='NAVIGATION',
               output_enabled=True,fault_latched=False,motion_command=[0.,0.,0.])
        self.assertEqual(locomotion_stopped(yaml.safe_dump(d),1_100_000_000,.25),1.)
        for changes in ({'observed_ns':0},{'phase':'RL_ACTIVE'},{'motion_command':[.1,0,0]},
                        {'fault_latched':True},{'motion_command':[math.nan,0,0]}):
            self.assertIsNone(locomotion_stopped(yaml.safe_dump(dict(d,**changes)),1_100_000_000,.25))

if __name__=='__main__':unittest.main()
