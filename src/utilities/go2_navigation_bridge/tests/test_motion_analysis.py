import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from nav_motion_analyze import analyze, manipulation_windows, interval_metrics, vehicle_velocity


class MotionAnalysisTests(unittest.TestCase):
    def test_reference_point_rotation_and_cross_covariance(self):
        # Sensor 0.3m in front of vehicle, pure rotation about stationary vehicle.
        v=np.array([0., .3, 0., 0., 0., 1.]); p=np.eye(6)*.01
        p[1,5]=p[5,1]=.004
        actual,cov=vehicle_velocity(v,p,[-.3,0,0])
        np.testing.assert_allclose(actual[:3],0.,atol=1e-12)
        self.assertAlmostEqual(cov[1,1], .01+.09*.01-2*.3*.004)
        p[0,0]=-1.
        with self.assertRaises(ValueError):vehicle_velocity(v,p,[-.3,0,0])

    def test_persistent_boundaries_survive_skipped_status_and_return_revision(self):
        rows=[dict(session_id='s',goal_revision=1,state='GRASP_PRESUMED_SUCCESS',observed_ns=10,
                   base_motion_window=dict(scan_started_ns=2,home_completed_ns=9)),
              dict(session_id='s',goal_revision=2,state='RETURN_NAVIGATING',observed_ns=11,
                   base_motion_window=dict(scan_started_ns=2,home_completed_ns=9))]
        windows=manipulation_windows(rows)
        self.assertEqual(len(windows),1);self.assertEqual(windows[0]['end_ns'],9)

    def test_incomplete_home_does_not_become_finished(self):
        windows=manipulation_windows([dict(session_id='s',state='BLOCKED',observed_ns=9,
            base_motion_window=dict(scan_started_ns=2,home_completed_ns=None))])
        self.assertIsNone(windows[0]['end_ns'])

    def fixture(self, folder, velocity=True, reset=False, owner_change=False):
        (folder/'navigation_profile.yaml').write_text(yaml.safe_dump(dict(robot=dict(sensor_offset_x_m=.3,sensor_offset_y_m=0.))))
        rows=[];vel=[]
        for i in range(81):
            t=1_000_000_000+i*50_000_000;h=t+100_000_000_000
            if reset and i>=40:h-=2_000_000_000
            rows.append(dict(receipt_mono_ns=t+30_000_000,receipt_ros_ns=h+30_000_000,clock_pair_span_ns=100,
                header_ns=h,frame='camera_init',child='aft_mapped',finite=True,x=.3+.2*i*.05,y=0,z=0,qx=0,qy=0,qz=0,qw=1,yaw=0))
            vel.append(dict(header_ns=h,frame='aft_mapped',vx=.2,vy=0,vz=0,wx=0,wy=0,wz=0,
                **{f'cov_{k}':.0001 if k%7==0 else 0. for k in range(36)}))
        for filename,data in [('odometry.csv',rows),*([('velocity.csv',vel)] if velocity else [])]:
            with (folder/filename).open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=data[0]);writer.writeheader();writer.writerows(data)
        events=[]
        for i,row in enumerate(rows):
            t=int(row['receipt_mono_ns'])-30_000_000
            events.append(dict(mono_ns=t,topic='/cmd_vel',frame='vehicle',command=[.15,0,0]))
            state='NAVIGATING' if i<20 else 'ARM_SCAN_POSE' if i<60 else 'GRASP_PRESUMED_SUCCESS'
            events.append(dict(mono_ns=t,topic='/go2/mission/status',data=dict(session_id='m',goal_revision=1,
                observed_ns=t,state=state,grasp_assessment='GRASPED' if i>=60 else 'UNKNOWN',
                base_motion_window=dict(scan_started_ns=2_000_000_000 if i>=20 else None,
                                        home_completed_ns=4_000_000_000 if i>=60 else None))))
            events.append(dict(mono_ns=t,topic='/go2/navigation/status',data=dict(session_id='n')))
        events.append(dict(mono_ns=1_000_000_000,topic='source_owners',data={'/state_estimation':['a']}))
        if owner_change:events.append(dict(mono_ns=3_000_000_000,topic='source_owners',data={'/state_estimation':['b']}))
        (folder/'status.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in events))

    def test_known_displacement_and_command_tracking_are_separate_from_accuracy(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);self.fixture(folder);result=analyze(folder)
            window=result['manipulation_windows'][0]
            self.assertEqual(window['metrics']['status'],'RELATIVE_ODOMETRY_ONLY')
            self.assertAlmostEqual(window['metrics']['displacement_xy_m'],.4)
            self.assertEqual(result['absolute_velocity_error'],'NOT_MEASURED')
            self.assertAlmostEqual(result['navigation_comparison']['before_grasp']['command_tracking_rmse_xy_mps'],.05)
            self.assertEqual(result['velocity_status'],'EKF_RECORDED')

    def test_missing_velocity_is_unknown_never_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);self.fixture(folder,velocity=False);result=analyze(folder)
            self.assertEqual(result['velocity_status'],'NOT_RECORDED')
            self.assertIsNone(result['navigation_comparison']['before_grasp']['estimated_speed_xy_mps']['mean'])

    def test_reset_or_new_publisher_invalidates_relative_interval(self):
        for options in [dict(reset=True),dict(owner_change=True)]:
            with self.subTest(options=options),tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);self.fixture(folder,**options);result=analyze(folder)
                self.assertEqual(result['manipulation_windows'][0]['metrics']['status'],'INVALID_INTERVAL')
                self.assertTrue(result['continuity_issues'])


if __name__=='__main__':unittest.main()
