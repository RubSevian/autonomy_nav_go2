import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_profile import load_profile, component_parameters
from navigation_goal import GoalTracker

class ProfileGoalTest(unittest.TestCase):
    def test_nominal_profile(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        self.assertEqual(profile['robot']['length_m'],.62)
        self.assertEqual(profile['local']['follower_motion_model'],'holonomic')
        self.assertFalse(profile['local']['narrow_recovered_selection'])
        self.assertEqual(profile['calibration']['rotation_speed_rps'],.7)

    def test_incompatible_geometry_and_limits(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        for section,key,value in [('calibration','rotation_speed_rps',0.),('calibration','rotation_speed_rps',2.),('robot','width_m',1.),('bridge','wz_limit',.5),('bridge','command_timeout_s',.5),('local','stop_distance_m',.3),('local','search_radius_m',float('nan')),('local','narrow_passage','false')]:
            changed=copy.deepcopy(profile);changed[section][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError):load_profile(file.name)

    def test_invalid_pairing_profile_settings(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        for key,value in [('tf_pair_history_s',.4),('tf_pair_max_samples',True),
                          ('tf_pair_max_samples',1.5),('tf_pair_max_samples',0),
                          ('tf_pair_position_tolerance_m',float('nan')),
                          ('tf_pair_quaternion_tolerance',0.)]:
            changed=copy.deepcopy(profile);changed['bridge'][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError,msg=key):load_profile(file.name)

    def test_invalid_pointlio_queue_settings(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        for key,value in [('queues.lidar_subscription_depth',0),('queues.imu_subscription_depth',True),
                          ('queues.lidar_buffer_capacity',1),('queues.imu_buffer_capacity',4001),
                          ('timing.status_period_s',0.),('timing.warn_age_s',float('nan')),
                          ('odometry.max_publish_rate_hz',float('nan')),
                          ('odometry.max_publish_rate_hz',-1.),
                          ('timing.status_topic','/different_recorder_topic')]:
            changed=copy.deepcopy(profile);changed['nodes']['laserMapping'][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError,msg=key):load_profile(file.name)

    def test_duplicate_cancel_and_new_goal(self):
        g=GoalTracker();self.assertTrue(g.accept((1.,0.,0.),'map',100,1.))
        self.assertFalse(g.accept((1.,0.,0.),'map',101,2.))
        self.assertEqual(g.revision,1)
        g.active();g.finish('CANCELLED',3.)
        g.reset();self.assertTrue(g.accept((1.,0.,0.),'map',200,4.))
        self.assertEqual(g.revision,2)

    def test_pointlio_unique_initialization_count(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        self.assertEqual(component_parameters(profile)['laserMapping']['mapping.imu_init_samples'],100)
        for value in (0,-1,True,1.5,None):
            changed=copy.deepcopy(profile)
            changed['nodes']['laserMapping']['mapping.imu_init_samples']=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError):load_profile(file.name)

    def test_invalid_sensor_time_settings(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        edits=[('time_warmup_s',0.),('time_margin_s',float('nan')),('time_status_period_s',0.),
               ('time_min_cloud_samples',1.5),('time_min_imu_samples',True),
               ('time_max_age_s',.6),('time_max_scan_duration_s',.5),
               ('time_status_topic','/utlidar/cloud')]
        for key,value in edits:
            changed=copy.deepcopy(profile);changed['sensors'][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError,msg=key):load_profile(file.name)

    def test_no_duplicate_derived_values_or_wrong_sensor_contract(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        edits=[('pathFollower','maxSpeed',.5),('pathFollower','sendSportCommand',True),
               ('laserMapping','mapping.imu_en',False),('far_planner','robot_dim',.01),
               ('far_planner','opencv_threads',0),('far_planner','opencv_threads',True),
               ('far_planner','opencv_threads',17),
               ('terrainAnalysis','worldFrame','odom'),('laserMapping','mapping.extrinsic_R',[1.])]
        for section,key,value in edits:
            changed=copy.deepcopy(profile);changed['nodes'][section][key]=value
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
                yaml.safe_dump(changed,file);file.flush()
                with self.assertRaises(ValueError,msg=f'{section}.{key}'):load_profile(file.name)

    def test_common_values_propagate_without_mutating_source(self):
        profile=load_profile(Path(__file__).resolve().parents[1]/'config/real_navigation.yaml')
        profile['robot']['sensor_offset_x_m']=.31
        profile['local']['goal_tolerance_m']=.3
        nodes=component_parameters(profile)
        self.assertAlmostEqual(nodes['pathFollower']['maxYawRate'],5.7295779513)
        self.assertEqual(nodes['localPlanner']['sensorOffsetX'],.31)
        self.assertEqual(nodes['far_planner']['sensor_offset_x'],.31)
        self.assertEqual(nodes['localPlanner']['goalCloseDis'],.3)
        self.assertEqual(nodes['far_planner']['g_planner/converge_distance'],.3)
        self.assertNotIn('sensorOffsetX',profile['nodes']['localPlanner'])

    def test_local_arrival_dwell_and_departure(self):
        g=GoalTracker();g.accept((1.,0.,0.),'map',100,1.)
        g.position(.8,0.,2.,True);g.position(.7,0.,2.4,True)
        self.assertEqual(g.state,'REQUESTED')
        g.position(.8,0.,3.,True);g.position(.8,0.,3.6,True)
        self.assertEqual(g.state,'SUCCEEDED')
        g.position(0.,0.,4.,True);self.assertEqual(g.state,'SUCCEEDED')

    def test_local_dwell_restarts_after_readiness_loss(self):
        g=GoalTracker();g.accept((1.,0.,0.),'map',100,1.)
        g.position(.9,0.,2.,True);g.position(.9,0.,2.4,False)
        g.position(.9,0.,3.,True)
        self.assertEqual(g.state,'REQUESTED')
        g.position(.9,0.,3.6,True)
        self.assertEqual(g.state,'SUCCEEDED')

    def test_far_requested_vs_adjusted_and_stale_status(self):
        g=GoalTracker();g.accept((1.,0.,0.),'map',100,1.)
        fields=dict(goal_original_x='1',goal_original_y='0',goal_original_z='0',reason_code='GOAL_REACHED')
        g.position(.5,0.,2.)
        self.assertFalse(g.far_status(fields,110,2.))
        g.position(.9,0.,3.)
        self.assertFalse(g.far_status(fields,99,3.))
        fields['goal_original_x']='2'
        self.assertFalse(g.far_status(fields,110,3.))
        fields['goal_original_x']='1';fields['reason_code']='ADJUSTED_GOAL_OUTSIDE_TOLERANCE'
        self.assertTrue(g.far_status(fields,110,3.))
        self.assertEqual(g.state,'STOPPED_ADJUSTED')

if __name__=='__main__':unittest.main()
