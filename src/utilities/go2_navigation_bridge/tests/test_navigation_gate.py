import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_gate import NavigationGate, ZERO

class GateTest(unittest.TestCase):
    def ready(self, mode='command_test'):
        g=NavigationGate(mode)
        g.locomotion(True,10.)
        for key in ('lidar','imu','odom','tf','path','stop'):g.observe(key,True,10.)
        g.planner_active(True,10.)
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
    def test_unknown_mode(self):
        with self.assertRaises(ValueError):NavigationGate('sim')

if __name__=='__main__':unittest.main()
