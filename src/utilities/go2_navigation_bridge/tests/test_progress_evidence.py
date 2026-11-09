"""Known trajectories validate the offline integral, frames and rejection rules."""
import ast
import csv
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from nav_progress_evidence import analyze


class EvidenceTests(unittest.TestCase):
    def capture(self, root, *, turn=False, inconsistent=False, gap=False, duplicate=False):
        (root/'navigation_profile.yaml').write_text(
            'robot:\n  sensor_offset_x_m: 0.3\n  sensor_offset_y_m: 0.0\n')
        with (root/'odometry.csv').open('w') as pose, (root/'velocity.csv').open('w') as velocity:
            pw=csv.writer(pose);vw=csv.writer(velocity)
            pw.writerow(['header_ns','receipt_ros_ns','clock_pair_span_ns','finite',
                         'frame','child','x','y','z','qx','qy','qz','qw'])
            vw.writerow(['header_ns','frame','vx','vy','vz','wx','wy','wz']+
                        [f'cov_{i}' for i in range(36)])
            for i in range(301):
                if gap and 40 <= i <= 100: continue
                t=i*.01; stamp=1_700_000_000_000_000_000+i*10_000_000
                yaw=t*.2 if turn else math.pi/2
                # Sensor fixed during turn; otherwise travels in world Y.
                pw.writerow([stamp,stamp+10_000_000,10,True,'camera_init','aft_mapped',
                             0.,0. if turn or inconsistent else .02*t,0.,
                             0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
                row=[stamp,'aft_mapped',0. if turn else .02,0.,0.,0.,0.,.2 if turn else 0.]+[
                    .001 if j%7==0 else 0. for j in range(36)]
                vw.writerow(row)
                if duplicate and i==50: vw.writerow(row)

    def test_translation_rotated_frame_and_turn_with_lever_arm(self):
        for turn in (False,True):
            with tempfile.TemporaryDirectory() as d:
                p=Path(d);self.capture(p,turn=turn)
                r=analyze(p)
                for window in r['windows'].values():
                    self.assertGreater(window['consistency_residual_m']['samples'],0)
                    self.assertLess(window['consistency_residual_m']['max'],1e-6)
                self.assertFalse(r['suitable_for_threshold_selection'])

    def test_detects_inconsistent_velocity_without_calling_it_physical_error(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.capture(p,inconsistent=True)
            r=analyze(p)
            self.assertAlmostEqual(r['windows']['1.0']['consistency_residual_m']['median'],.02)
            self.assertFalse(r['ground_truth']);self.assertFalse(r['stationary_confirmed'])

    def test_missing_duplicate_gap_and_partial_rows(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.capture(p,gap=True,duplicate=True)
            with (p/'velocity.csv').open('a') as f: f.write('partial\n')
            r=analyze(p)
            self.assertEqual(r['invalid_velocity_rows'],1)
            self.assertEqual(r['discontinuities'],1)
            (p/'odometry.csv').unlink()
            self.assertEqual(analyze(p)['missing'],['odometry.csv'])
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.capture(p,duplicate=True)
            r=analyze(p)
            self.assertEqual(r['duplicate_velocity_stamps'],1)
            self.assertEqual(r['unmatched_pose_rows'],1)

    def test_real_recorder_callback_writes_pose_before_shutdown(self):
        # Execute the actual callback, without importing ROS or creating a node.
        tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/nav_drift.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        callback=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='odom')
        callback.body=[n for n in callback.body if not isinstance(n,ast.Nonlocal)]
        module=ast.fix_missing_locations(ast.Module(body=[callback],type_ignores=[]))
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'odometry.csv'
            with p.open('w',buffering=1) as stream:
                ns=dict(math=math,rows=[],config={'pose_capacity':10},phase='test',
                        events=[],clocks=lambda:(1,2,3),count=lambda *a:None,
                        odometrywriter=csv.writer(stream))
                exec(compile(module,'nav_drift_callback','exec'),ns)
                msg=NS(header=NS(stamp=NS(sec=1,nanosec=0),frame_id='camera_init'),
                       child_frame_id='aft_mapped',pose=NS(pose=NS(
                       position=NS(x=1.,y=2.,z=3.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
                ns['odom'](msg,dict(source_timestamp=1,received_timestamp=2))
                with p.open() as recorded:
                    self.assertEqual(len(list(csv.reader(recorded))),1)
                self.assertEqual(ns['rows'][0][5:8],[1.,2.,3.])


if __name__=='__main__': unittest.main()
