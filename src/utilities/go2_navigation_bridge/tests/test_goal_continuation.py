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

    def test_unconfirmed_stop_has_distinct_final_reason(self):
        g=self.goal(wait_budget_s=1.)
        for i in range(12):self.position(g,.12,-.31,2.+i*.1,1000+i,False)
        self.assertEqual(g.state,'ARRIVAL_NOT_CONFIRMED')

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

    def test_same_stamp_pose_revision_can_break_but_not_advance_dwell(self):
        g=self.goal();self.position(g,.12,-.31,2.,1000)
        self.position(g,.12,-.31,2.2,1100)
        self.position(g,0.,0.,2.3,1100) # revised outside A, same measurement stamp
        self.assertEqual(g.state,'REPLANNING');self.assertIsNone(g.inside_since)
        self.position(g,.12,-.31,2.4,1200)
        self.position(g,.12,-.31,2.6,1300)
        self.assertNotEqual(g.state,'SUCCEEDED')
        self.position(g,.15,-.31,2.7,1300) # revision moved inside A, breaks anchor
        self.assertIsNone(g.inside_since)

    def test_external_tf_expiry_restarts_confirmation_without_old_anchor(self):
        g=self.goal();self.position(g,.12,-.31,2.,1000)
        g.inside_since=None # same revocation performed by the TF expiry guard
        self.position(g,.12,-.31,2.2,1100)
        self.assertNotEqual(g.state,'SUCCEEDED')
        self.assertEqual(g.inside_since,2.2)

    def test_profile_rejects_inconsistent_wait_and_settle_limits(self):
        import tempfile,yaml
        from navigation_profile import load_profile
        path=Path(__file__).resolve().parents[1]/'config/real_navigation.yaml'
        for key,value in (('goal_wait_budget_s',.5),('goal_progress_noise_bound_m',-.01),('goal_settle_position_m',.25),
                          ('goal_settle_yaw_rad',4.),('goal_wait_budget_s',True)):
            profile=load_profile(path);profile['bridge'][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as f:
                yaml.safe_dump(profile,f);f.flush()
                with self.assertRaises(ValueError):load_profile(f.name)

    def moving_goal(self, route=((0.,0.),(2.,0.)), calibrated=True):
        import json
        g=self.goal(progress_noise_bound_m=.008 if calibrated else 0.)
        self.status(g,'ADJUSTED_GOAL_WAIT',110,2.)
        self.status(g,'ROUTE_COMMITTED',10_000_000_000,10.,navigation_active='true',
                    committed_route_xy=json.dumps(route),committed_route_frame='map')
        self.assertEqual(g.wait_spent,8.)
        return g

    def travel(self,g,i,x,y=0.,motion=(.2,0.,0.),stamp=None):
        t=10.+i*.05;stamp=10_000_000_000+i*50_000_000 if stamp is None else stamp
        # A successful FAR heartbeat refreshes route evidence, not waiting credit.
        if i%10==0 and g.progress.route is not None:
            g.progress.set_route(g.progress.routes[g.progress.route]['points'],stamp)
        return g.observe_progress(x,y,0.,stamp,t,t,motion)

    def test_default_episode_has_ten_seconds(self):
        g=self.goal();self.status(g,'ADJUSTED_GOAL_WAIT',110,2.)
        g.advance(11.99);self.assertEqual(g.state,'REPLANNING')
        g.advance(12.);self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')
        from navigation_profile import load_profile
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        self.assertEqual(profile['bridge']['goal_wait_budget_s'],10.)
        self.assertAlmostEqual(profile['bridge']['goal_progress_noise_bound_m'],.0342)

    def test_measured_small_progress_rearms_without_erasing_history(self):
        g=self.moving_goal()
        credited=[self.travel(g,i,.001*i) for i in range(40)]
        self.assertTrue(any(credited));self.assertEqual(g.wait_spent,0.)
        self.assertEqual(g.wait_total,8.);self.assertEqual(g.progress_resets,1)
        self.status(g,'ADJUSTED_GOAL_WAIT',13_000_000_000,13.)
        g.advance(22.99);self.assertEqual(g.state,'REPLANNING')
        g.advance(23.);self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')

    def test_unknown_noise_does_not_silently_fallback_to_ten_centimetres(self):
        g=self.moving_goal(calibrated=False)
        for i in range(60):self.travel(g,i,.005*i)
        self.assertEqual(g.wait_spent,8.)
        self.assertEqual(g.snapshot(13.)['progress']['motion']['reason'],'UNCALIBRATED')

    def test_missing_local_path_consumes_same_episode(self):
        g=self.moving_goal();g.path_waiting=True
        g.advance(11.99);self.assertEqual(g.state,'ACTIVE')
        g.advance(12.);self.assertEqual(g.state,'CONTINUATION_NOT_FOUND')

    def test_detour_can_rearm_while_distance_to_A_grows(self):
        g=self.moving_goal(route=((0.,0.),(0.,2.)))
        credited=[self.travel(g,i,0.,.001*i,(0.,.2,0.)) for i in range(40)]
        self.assertTrue(any(credited));self.assertEqual(g.wait_spent,0.)

    def test_idle_rotation_wrong_direction_and_jitter_do_not_rearm(self):
        for motion in (None,(0.,0.,.1),(-.2,0.,0.)):
            g=self.moving_goal()
            for i in range(60):self.travel(g,i,.001*i,motion=motion)
            self.assertEqual(g.wait_spent,8.)
        g=self.moving_goal()
        for i in range(60):self.travel(g,i,.002*(i%2))
        self.assertEqual(g.wait_spent,8.)

    def test_duplicate_jump_gap_and_stale_route_do_not_rearm(self):
        for case in ('duplicate','jump','gap','stale_route'):
            g=self.moving_goal();self.travel(g,0,0.)
            for i in range(1,60):
                stamp=10_000_000_000 if case=='duplicate' else 10_000_000_000+i*50_000_000
                if case=='gap':stamp+=i*300_000_000
                x=.3 if case=='jump' else .001*i
                if case=='stale_route':g.progress.route_stamp=1
                # Bypass the heartbeat helper for the stale-route scenario.
                g.observe_progress(x,0.,0.,stamp,10.+i*.05,10.+i*.05,(.2,0.,0.))
            self.assertEqual(g.wait_spent,8.,case)

    def test_repeat_failure_is_terminal_and_new_goal_clears_history(self):
        g=self.moving_goal();g.progress.failure='REPEATED_ATTEMPT';g.advance(10.1)
        self.assertEqual(g.state,'REPEATED_ATTEMPT')
        self.status(g,'ROUTE_COMMITTED',11_000_000_000,11.)
        self.assertEqual(g.state,'REPEATED_ATTEMPT')
        self.assertTrue(g.accept(g.target,'map',12_000_000_000,12.))
        self.assertEqual(g.progress.routes,[]);self.assertIsNone(g.progress.failure)

    def test_r3_zero_requires_fresh_valid_report(self):
        import yaml
        d=dict(observed_ns=1_000_000_000,state='ACTIVE',phase='RL_ZERO',command_source='NAVIGATION',
               output_enabled=True,fault_latched=False,motion_command=[0.,0.,0.])
        self.assertEqual(locomotion_stopped(yaml.safe_dump(d),1_100_000_000,.25),1.)
        for changes in ({'observed_ns':0},{'phase':'RL_ACTIVE'},{'motion_command':[.1,0,0]},
                        {'fault_latched':True},{'motion_command':[math.nan,0,0]}):
            self.assertIsNone(locomotion_stopped(yaml.safe_dump(dict(d,**changes)),1_100_000_000,.25))

if __name__=='__main__':unittest.main()
