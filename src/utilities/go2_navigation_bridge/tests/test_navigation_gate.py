import sys
import unittest
from types import SimpleNamespace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_gate import NavigationGate, ZERO
from navigation_contract import valid_cloud_layout, locomotion_ready

class GateTest(unittest.TestCase):
    def ready(self, mode='command_test'):
        g=NavigationGate(mode)
        g.locomotion(True,10.)
        for key in ('lidar','imu','odom','tf','path','stop'):g.observe(key,True,10.)
        g.planner_active(True,10.);g.goal()
        return g
    def test_directions_and_clamp(self):
        for axis in range(3):
            for sign in (-1,1):
                g=self.ready();v=[0.,0.,0.];v[axis]=sign*9
                self.assertTrue(g.accept(v,0,'vehicle',10.))
                self.assertEqual(g.output(10.),tuple(sign*(.2,.1,.1)[i] if i==axis else 0. for i in range(3)))
    def test_header_and_receive_deadline(self):
        g=self.ready();g.accept((.1,0,0),.2,'vehicle',10.)
        self.assertNotEqual(g.output(10.04),ZERO)
        self.assertEqual(g.output(10.06),ZERO)
    def test_invalid_revokes_previous(self):
        for value,age,frame in [((float('nan'),0,0),0,'vehicle'),((.1,0,0),-.01,'vehicle'),((.1,0,0),.26,'vehicle'),((.1,0,0),0,'map')]:
            g=self.ready();g.accept((.1,0,0),0,'vehicle',10.)
            self.assertFalse(g.accept(value,age,frame,10.01));self.assertEqual(g.output(10.01),ZERO)
    def test_cancel_resume_no_replay(self):
        g=self.ready();g.accept((.1,0,0),0,'vehicle',10.)
        g.cancel();self.assertEqual(g.output(10.),ZERO)
        self.assertFalse(g.accept((.1,0,0),0,'vehicle',10.))
        g.resume();self.assertEqual(g.output(10.),ZERO)
        self.assertTrue(g.accept((.1,0,0),0,'vehicle',10.01))
    def test_sensor_dropout_not_emergency(self):
        for key in ('lidar','imu','odom','tf'):
            g=self.ready('stack');g.accept((.1,0,0),0,'vehicle',10.)
            g.observe(key,False,10.01);self.assertEqual(g.output(10.01),ZERO)
            g.observe(key,True,10.02);self.assertEqual(g.output(10.02),ZERO)
    def test_stale_localization(self):
        g=self.ready('stack');g.locomotion(True,10.6);g.planner_active(True,10.6)
        self.assertFalse(g.localization_ready(10.6));self.assertEqual(g.output(10.6),ZERO)
    def test_path_stop_and_active(self):
        for kind in ('path','stop','active'):
            g=self.ready('stack');g.accept((.1,0,0),0,'vehicle',10.)
            if kind=='active':g.planner_active(False,10.01)
            else:g.observe(kind,False,10.01)
            self.assertEqual(g.output(10.01),ZERO)
    def test_fsm_not_active(self):
        g=self.ready();g.accept((.1,0,0),0,'vehicle',10.)
        g.locomotion(False,10.01);self.assertEqual(g.output(10.01),ZERO)
        g.locomotion(True,10.02);self.assertEqual(g.output(10.02),ZERO)
    def test_readiness_independent_of_leg_activation(self):
        g=NavigationGate('command_test');self.assertTrue(g.localization_ready(10.))
        self.assertFalse(g.accept((.1,0,0),0,'vehicle',10.))
    def test_source_age_not_renewed_by_receipt(self):
        g=self.ready('stack')
        g.observe('odom',True,10.,.4)
        self.assertFalse(g.localization_ready(10.11))
    def test_resume_requires_new_goal_and_preserves_external_stop(self):
        g=self.ready('stack');g.observe('stop',False,10.)
        g.cancel();g.resume()
        g.observe('path',True,10.01)
        self.assertFalse(g.enabled(10.01))
        g.goal();self.assertFalse(g.enabled(10.01))
        g.observe('stop',True,10.02);self.assertTrue(g.enabled(10.02))
    def test_heartbeat_must_not_be_refreshed_from_cached_intent(self):
        g=self.ready('stack')
        for name in ('lidar','imu','odom','tf','path'):g.observe(name,True,10.6)
        g.locomotion(True,10.6)
        self.assertFalse(g.enabled(10.6));self.assertEqual(g.block_reason,'planner_inactive')
    def test_configured_freshness_applies_to_path_heartbeat_and_locomotion(self):
        for stale,reason in [('path','path_unavailable'),('heartbeat','planner_inactive'),('locomotion','locomotion_not_active')]:
            g=NavigationGate('stack',sensor_timeout=.1)
            for key in ('lidar','imu','odom','tf','path','stop'):g.observe(key,True,10.11)
            g.goal();g.locomotion(True,10.11);g.planner_active(True,10.11)
            self.assertTrue(g.enabled(10.11))
            if stale=='path':g.observe('path',True,10.)
            elif stale=='heartbeat':g.planner_active(True,10.)
            else:g.locomotion(True,10.)
            self.assertFalse(g.enabled(10.11));self.assertEqual(g.block_reason,reason)
    def test_cloud_layout_rejects_corrupt_structure(self):
        fields=[SimpleNamespace(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(('x','y','z','intensity','time'))]
        msg=SimpleNamespace(fields=fields,width=1,height=1,point_step=20,row_step=20,data=bytes(20))
        self.assertTrue(valid_cloud_layout(msg))
        msg.row_step=19;self.assertFalse(valid_cloud_layout(msg));msg.row_step=20
        fields[0].datatype=2;self.assertFalse(valid_cloud_layout(msg));fields[0].datatype=7
        fields[-1].offset=20;self.assertFalse(valid_cloud_layout(msg));fields[-1].offset=16
        fields[-1].name='t';self.assertFalse(valid_cloud_layout(msg));fields[-1].name='time'
        fields[-1].datatype=6;self.assertFalse(valid_cloud_layout(msg));fields[-1].datatype=7
        fields.append(fields[0]);self.assertFalse(valid_cloud_layout(msg))
    def test_cancel_rpc_keeps_one_future_and_backs_off(self):
        import ast
        source=ast.parse((Path(__file__).resolve().parents[1]/'scripts/go2_navigation_bridge.py').read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name == 'NavigationBridge')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='cancel_far_tick')
        ns={'Trigger':SimpleNamespace(Request=lambda:None)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'<actual cancel_far_tick>','exec'),ns)
        sent=[];removed=[]
        def send(request):
            future=SimpleNamespace(done=lambda:False);sent.append(future);return future
        state=SimpleNamespace(cancel_retry=.5,cancel_timeout=1.,far_cancel_future=None,far_cancel_pending=True,far_cancel_started=0.,far_cancel_retry_at=0.,far_cancel_client=SimpleNamespace(service_is_ready=lambda:True,call_async=send,remove_pending_request=removed.append))
        tick=ns['cancel_far_tick']
        tick(state,0.)
        for i in range(1,100):tick(state,i*.01)
        self.assertEqual(len(sent),1)
        tick(state,1.1);self.assertEqual(len(removed),1)
        tick(state,1.2);self.assertEqual(len(sent),1)
        tick(state,1.61);self.assertEqual(len(sent),2)
    def test_unknown_mode(self):
        with self.assertRaises(ValueError):NavigationGate('sim')

class LocomotionContractTest(unittest.TestCase):
    def payload(self, **changes):
        import yaml
        status = dict(state='ACTIVE', phase='RL_ZERO', output_enabled=True,
                      fault_latched=False, command_source='NAVIGATION')
        status.update(changes)
        return yaml.safe_dump(status)

    def test_active_navigation_and_remote_are_distinct(self):
        for phase in ('RL_ZERO', 'RL_ACTIVE'):
            self.assertTrue(locomotion_ready(self.payload(phase=phase)))
            self.assertFalse(locomotion_ready(self.payload(phase=phase, command_source='REMOTE')))

    def test_each_readiness_condition_can_revoke(self):
        for changes in (dict(state='STANDBY'), dict(phase='HOLDING'),
                        dict(output_enabled=False), dict(fault_latched=True)):
            with self.subTest(changes=changes):
                self.assertFalse(locomotion_ready(self.payload(**changes)))

    def test_malformed_missing_fields_and_unsafe_tags_revoke(self):
        for payload in ('', 'null', '[]', '42', '{}', '{state: ACTIVE}',
                        '{broken:', '!!python/object/apply:builtins.str [123]'):
            with self.subTest(payload=payload):
                self.assertFalse(locomotion_ready(payload))

    def test_safe_python_fallback_preserves_contract(self):
        import yaml
        from unittest.mock import patch
        with patch('navigation_contract._SAFE_LOADER', yaml.SafeLoader):
            self.assertTrue(locomotion_ready(self.payload()))
            self.assertFalse(locomotion_ready(self.payload(fault_latched=True)))

if __name__=='__main__':unittest.main()
