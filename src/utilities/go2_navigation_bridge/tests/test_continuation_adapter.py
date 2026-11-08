"""Actual bridge tick and callbacks on fake I/O ports: no node or DDS init."""
from pathlib import Path
import sys
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2_navigation_bridge import NavigationBridge
from navigation_goal import GoalTracker
from navigation_gate import NavigationGate
from navigation_recovery import PathRecovery


class ContinuationAdapterTests(unittest.TestCase):
    def node(self):
        gate=NavigationGate()
        for name in ('lidar','imu','odom','tf','path','stop'):gate.observe(name,True,100.)
        gate.locomotion(True,100.);gate.planner_active(True,100.);gate.goal()
        self.assertTrue(gate.accept((.1,0.,0.),0.,'vehicle',100.))
        goal=GoalTracker(far_mode=True);goal.accept((1,0,0),'map',10,99.)
        goal.state='REPLANNING'
        out=[];ready=[]
        n=NS(gate=gate,goals=goal,last_tick_started_ns=None,last_tf_check=100.,tf_check_period=1.,
             pairing=NS(selected=None),local_route=False,far_route=True,active_hint=True,
             last_graph_check=100.,graph_check_period=1.,active_owner_gid=b'x',session='nav',
             active_owner_since_ns=0,active_observed_ns=0,active_session='',
             last_status_publish=100.,status_period=1.,path_recovery=PathRecovery(),path_recovery_state='READY',
             mission_health=NS(graph=lambda *a:None,base_locked=False,return_request_ns=0,valid=lambda *a:True),
             get_publishers_info_by_topic=lambda *a:[],get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
             cancel_far_tick=lambda *a:None,cmd_pub=NS(publish=out.append),ready_pub=NS(publish=ready.append),
             command_after_ns=10,goal_tf_valid=True,header_age=lambda msg:0.,header_valid=lambda msg:True,
             far_cancel_pending=False,far_cancel_future=None)
        return n,out,ready

    def tick(self,n):
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.),patch('go2_navigation_bridge.time.monotonic_ns',return_value=100_000_000_000):
            NavigationBridge.tick(n)

    def test_pause_revokes_cached_velocity_and_path(self):
        n,out,_=self.node();self.tick(n)
        self.assertEqual(out[-1].twist.linear.x,0.)
        self.assertFalse(n.gate.request_received)
        self.assertIsNone(n.gate.health['path'])
        self.assertTrue(n.gate.goal_received)
        self.assertEqual(n.goals.state,'REPLANNING')

    def test_resume_needs_new_path_and_new_velocity(self):
        n,out,_=self.node();self.tick(n)
        n.goals.state='ACTIVE';self.tick(n)
        self.assertEqual(out[-1].twist.linear.x,0.)
        self.assertEqual(n.path_recovery_state,'WAITING_FOR_PATH')
        p=NS(pose=NS(position=NS(x=0.,y=0.,z=0.)))
        msg=NS(header=NS(frame_id='vehicle',stamp=NS(sec=100,nanosec=0)),poses=[p,p])
        NavigationBridge.path(n,msg)
        self.assertIsNone(n.gate.health['path']) # equal-to-pause stamp is old
        msg.header.stamp.nanosec=1
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.):NavigationBridge.path(n,msg)
        self.tick(n)
        self.assertEqual(out[-1].twist.linear.x,0.) # path recovery is not velocity replay
        cmd=NS(header=NS(frame_id='vehicle',stamp=NS(sec=100,nanosec=1)),
               twist=NS(linear=NS(x=.1,y=0.),angular=NS(z=0.)))
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.):NavigationBridge.command(n,cmd)
        self.tick(n);self.assertEqual(out[-1].twist.linear.x,.1)

    def test_authority_loss_while_paused_revokes_task(self):
        n,out,_=self.node();n.gate.locomotion(False,100.);self.tick(n)
        self.assertEqual(n.goals.state,'AUTHORITY_LOST')
        self.assertFalse(n.gate.goal_received);self.assertTrue(n.far_cancel_pending)
        self.assertEqual(out[-1].twist.linear.x,0.)

    def test_wait_timeout_stops_and_schedules_far_cancel(self):
        n,out,_=self.node();n.goals.wait_spent=n.goals.wait_budget_s;self.tick(n)
        self.assertEqual(n.goals.state,'CONTINUATION_NOT_FOUND')
        self.assertFalse(n.gate.goal_received);self.assertTrue(n.far_cancel_pending)
        self.assertEqual(out[-1].twist.linear.x,0.)

    def test_stop_heartbeat_accounts_pause_before_diagnostic_arrives(self):
        n,out,_=self.node();n.goals.state='ACTIVE'
        msg=NS(data=json.dumps(dict(observed_ns=99_990_000_000,session_id='far',active=False)))
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.),patch('go2_navigation_bridge.time.monotonic_ns',return_value=100_000_000_000):
            NavigationBridge.active(n,msg)
        self.assertEqual(n.goals.state,'REPLANNING')
        self.tick(n);self.assertEqual(out[-1].twist.linear.x,0.)

    def test_delayed_stop_heartbeat_cannot_undo_newer_route_commit(self):
        n,_,_=self.node();n.far_status_timeout=1.5;n.header_age=lambda msg:.01
        fields=dict(goal_original_x='1',goal_original_y='0',goal_original_z='0',goal_stamp_ns='10',reason_code='ROUTE_COMMITTED')
        msg=NS(header=NS(stamp=NS(sec=100,nanosec=0)),status=[NS(name='far_planner',values=[NS(key=k,value=v) for k,v in fields.items()])])
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.),patch('go2_navigation_bridge.time.monotonic_ns',return_value=100_000_000_000):
            NavigationBridge.far_status(n,msg)
            self.assertEqual(n.goals.state,'ACTIVE')
            NavigationBridge.active(n,NS(data=json.dumps(dict(observed_ns=99_980_000_000,session_id='far',active=False))))
            self.assertEqual(n.goals.state,'ACTIVE')
            self.assertFalse(n.gate.active) # Still no movement until the fresh active heartbeat.
            NavigationBridge.active(n,NS(data=json.dumps(dict(observed_ns=99_995_000_000,session_id='far',active=True))))
            self.assertEqual(n.goals.state,'ACTIVE');self.assertTrue(n.gate.active)

    def test_r3_motion_report_immediately_breaks_arrival_window(self):
        import yaml
        n,_,_=self.node();n.goals.state='ARRIVAL_CONFIRMING'
        n.goals.inside_since=99.6;n.goals.settle_anchor=(1.,0.,0.)
        report=dict(state='ACTIVE',phase='RL_ACTIVE',command_source='NAVIGATION',
                    output_enabled=True,fault_latched=False,observed_ns=100_000_000_000,motion_command=[.1,0,0])
        with patch('go2_navigation_bridge.time.monotonic',return_value=100.),patch('go2_navigation_bridge.time.monotonic_ns',return_value=100_000_000_000):
            NavigationBridge.locomotion(n,NS(data=yaml.safe_dump(report)))
        self.assertIsNone(n.goals.inside_since);self.assertIsNone(n.goals.settle_anchor)

    def test_unconfirmed_arrival_cannot_leave_velocity_authorized(self):
        n,out,_=self.node();n.goals.state='ARRIVAL_NOT_CONFIRMED'
        self.tick(n)
        self.assertFalse(n.gate.goal_received);self.assertTrue(n.far_cancel_pending)
        self.assertEqual(out[-1].twist.linear.x,0.)

    def test_progress_observation_uses_only_actual_unmasked_navigation_motion(self):
        for locked in (False,True):
            n,out,_=self.node();n.goals.state='ACTIVE';n.goals.wait_spent=8.
            pair=NS(odom=NS(stamp_ns=100_000_000_000),vehicle=NS(position=(0.,0.,0.),quaternion=(0.,0.,0.,1.)))
            n.last_tf_check=0.;n.odometry_subscription=None;n.odom=lambda *a:None
            n.tf_listener=NS(tf_sub=None,callback=lambda *a:None)
            n.pairing=NS(selected=pair,select=lambda *a:pair,reason='')
            n.resolve_vehicle=lambda *a:None;n.stopped_observed=None;n.mission_health.base_locked=locked
            with patch('go2_navigation_bridge.drain_pending',return_value=0):self.tick(n)
            self.assertEqual(n.goals.progress_sample is None,locked)
            self.assertEqual(out[-1].twist.linear.x,0. if locked else .1)

    def test_wait_for_fresh_path_is_accounted_in_episode_budget(self):
        n,out,_=self.node();n.goals.state='ACTIVE';n.gate.health['path']=None
        self.tick(n)
        self.assertTrue(n.goals.path_waiting)
        self.assertEqual(out[-1].twist.linear.x,0.)

    def test_new_goal_is_rejected_while_old_cancel_is_pending(self):
        n,_,_=self.node();n.far_cancel_pending=True
        NavigationBridge.goal(n,None)
        self.assertEqual(n.goals.revision,1)

if __name__=='__main__':unittest.main()
