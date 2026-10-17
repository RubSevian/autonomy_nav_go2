"""Resolve launch actions without starting any node or ROS domain."""
import importlib.util
import os
import tempfile
import copy
import yaml
from pathlib import Path
from launch import LaunchContext
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='nav-launch-log-')
p=Path(__file__).resolve().parents[1]/'launch/real_navigation.launch.py'
spec=importlib.util.spec_from_file_location('nav_launch',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
source=p.parents[3]
packages={'point_lio_unilidar':source/'slam/point_lio_unilidar','local_planner':source/'base_autonomy/local_planner','terrain_analysis':source/'base_autonomy/terrain_analysis','terrain_analysis_ext':source/'base_autonomy/terrain_analysis_ext','go2_navigation_bridge':p.parents[1], 'far_planner':source/'route_planner/far_planner'}
m.get_package_share_directory=lambda key:str(packages[key])
captured=[];realnode=m.Node
m.Node=lambda **kw:captured.append(kw) or realnode(**kw)
with tempfile.NamedTemporaryFile(suffix='.yaml') as f:
    f.write(b'acc_bias_x: 0\nacc_bias_y: 0\nacc_bias_z: 0\nang_bias_x: 0\nang_bias_y: 0\nang_bias_z: 0\nang_z2x_proj: 0\nang_z2y_proj: 0\n');f.flush()
    def context(stage,path=f.name):
        c=LaunchContext();c.launch_configurations.update(stage=stage,observe_only='false',calibration_path=path,sensor_offset_x='0.3',sensor_offset_y='0.0',profile_path=str(p.parents[1]/'config/real_navigation.yaml'));return c
    for stage in ('command_test','localization','local','far'):
        captured.clear();actions=m._stack(context(stage));assert actions
        if stage != 'command_test':
            base=[x for x in captured if x.get('name')=='nav_sensor_vehicle']
            assert len(base)==1 and '--x' in base[0]['arguments']
        assert not any(x['package']=='unitree_legged_real' for x in captured)
        assert not any(x['package']=='calibrate_imu' for x in captured)
        assert not any('sport' in x['package'] for x in captured)
        by_name={node.get('name'):node for node in captured}
        if stage != 'command_test':
            slam=by_name['laserMapping']
            assert ('/path','/point_lio/path') in slam['remappings']
            assert slam['parameters'][0]['common.imu_topic']=='/utlidar/transformed_raw_imu'
            assert slam['parameters'][0]['use_imu_as_input'] is False
        if stage in ('local','far'):
            assert ('/cmd_vel','/navigation/cmd_vel') in by_name['pathFollower']['remappings']
            assert by_name['pathFollower']['parameters'][0]['sendSportCommand'] is False
            assert by_name['localPlanner']['parameters'][0]['goalCloseDis']==.25
        if stage=='far':
            assert ('/terrain_cloud','/terrain_map_ext') in by_name['far_planner']['remappings']
            assert by_name['far_planner']['parameters'][0]['g_planner/converge_distance']==.25
            recorder=by_name['manual_drift_recorder']
            assert recorder['executable']=='nav_drift.py'
            assert recorder['arguments']==['--config',str(p.parents[1]/'config/real_navigation.yaml')]
        else:
            assert 'manual_drift_recorder' not in by_name
    c=context('far');c.launch_configurations['observe_only']='true'
    captured.clear();m._stack(c)
    bridge=next(x for x in captured if x['package']=='go2_navigation_bridge')
    assert ('/cmd_vel','/diagnostics/manual_nav/cmd_vel') in bridge['remappings']
    assert ('/go2/mission/readiness','/diagnostics/manual_nav/readiness') in bridge['remappings']
    assert bridge['parameters'][0]['require_clock_alignment'] is True
    relay=next(x for x in captured if x['package']=='transform_sensors')
    assert relay['parameters'][0]['time_alignment_variant']=='bounded_common_v1'
    assert relay['parameters'][0]['time_status_period_s']==.1
    assert len([x for x in captured if x['package']=='point_lio_unilidar'])==1
    # Edits to ONE profile reach every consumer, without installed FAR/Point-LIO config files.
    changed=yaml.safe_load((p.parents[1]/'config/real_navigation.yaml').read_text())
    changed['bridge']['tf_pair_history_s']=1.5
    changed['bridge']['tf_pair_max_samples']=3072
    changed['bridge']['tf_pair_position_tolerance_m']=.002
    changed['bridge']['tf_pair_quaternion_tolerance']=.003
    changed['local']['goal_tolerance_m']=.30
    changed['sensors']['cloud_topic']='/audit/cloud'
    changed['sensors']['raw_imu_output_topic']='/audit/imu'
    changed['nodes']['laserMapping']['filter_size_surf']=.12
    changed['nodes']['terrainAnalysis']['decayTime']=3.
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as edited:
        yaml.safe_dump(changed,edited);edited.flush()
        c=context('far');c.launch_configurations['profile_path']=edited.name
        captured.clear();m._stack(c)
        by_name={node.get('name'):node for node in captured}
        for name in ('localPlanner','pathFollower'):
            assert by_name[name]['parameters'][0]['goalCloseDis']==.30
        assert by_name['far_planner']['parameters'][0]['g_planner/converge_distance']==.30
        bridge=next(node for node in captured if node['package']=='go2_navigation_bridge')['parameters'][0]
        assert bridge['goal_tolerance_m']==.30 and bridge['imu_topic']=='/audit/imu'
        assert bridge['tf_pair_history_s']==1.5 and bridge['tf_pair_max_samples']==3072
        assert type(bridge['tf_pair_max_samples']) is int
        assert bridge['tf_pair_position_tolerance_m']==.002 and bridge['tf_pair_quaternion_tolerance']==.003
        assert by_name['laserMapping']['parameters'][0]['common.lid_topic']=='/audit/cloud'
        assert by_name['transform_everything']['parameters'][0]['cloud_topic']=='/audit/cloud'
        assert by_name['terrainAnalysis']['parameters'][0]['decayTime']==3.
        assert by_name['laserMapping']['parameters'][0]['filter_size_surf']==.12
    for stage,path in [('bogus',f.name),('localization','/missing-calibration')]:
        try:m._stack(context(stage,path))
        except ValueError:pass
        else:raise AssertionError('bad stage/missing calibration accepted')
text=p.read_text()
assert 'IncludeLaunchDescription' not in text # no hidden XML or secondary YAML overrides
assert "far_route=stage == 'far'" in text
assert '/sim/' not in text and 'mujoco' not in text.lower()
print('PASS four stages, no locomotion/Sport node, topic remaps, single profile propagation, calibration fail-closed')

# Parse the actual XML condition/argument contract without executing its Nodes.
from launch.launch_description_sources import AnyLaunchDescriptionSource
desc=AnyLaunchDescriptionSource(str(packages['local_planner']/'launch/local_planner.launch')).get_launch_description(LaunchContext())
assert desc.entities
print('PASS actual local_planner XML with optional common base TF')

# Resolve actions only: NEVER execute the physical Sport calibrator.
spec=importlib.util.spec_from_file_location('calibration_launch',p.with_name('imu_calibration.launch.py'))
cal=importlib.util.module_from_spec(spec);spec.loader.exec_module(cal)
cal.get_package_share_directory=m.get_package_share_directory
cal.Node=m.Node
changed=yaml.safe_load((p.parents[1]/'config/real_navigation.yaml').read_text())
changed['calibration']['rotation_speed_rps']=.4
changed['sensors']['raw_imu_topic']='/audit/raw_imu'
with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
    yaml.safe_dump(changed,file);file.flush()
    context=LaunchContext();context.launch_configurations['profile_path']=file.name
    captured.clear();actions=cal._calibration(context)
    assert len(actions)==len(captured)==1
    node=captured[0]
    assert node['package']=='calibrate_imu' and node['parameters'][0]['rotation_speed_rps']==.4
    assert ('/utlidar/imu','/audit/raw_imu') in node['remappings']
print('PASS separate calibration launch uses profile speed/topic; no calibrator in NAV; nothing executed')
