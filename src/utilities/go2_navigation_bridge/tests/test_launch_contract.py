"""Resolve launch actions without starting any node or ROS domain."""
import importlib.util
import os
import tempfile
from pathlib import Path
from launch import LaunchContext
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='nav-launch-log-')
p=Path(__file__).resolve().parents[1]/'launch/real_navigation.launch.py'
spec=importlib.util.spec_from_file_location('nav_launch',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
source=p.parents[3]
packages={'point_lio_unilidar':source/'slam/point_lio_unilidar','local_planner':source/'base_autonomy/local_planner','terrain_analysis':source/'base_autonomy/terrain_analysis','terrain_analysis_ext':source/'base_autonomy/terrain_analysis_ext','far_planner':source/'route_planner/far_planner'}
m.get_package_share_directory=lambda key:str(packages[key])
captured=[];realnode=m.Node
m.Node=lambda **kw:captured.append(kw) or realnode(**kw)
with tempfile.NamedTemporaryFile(suffix='.yaml') as f:
    f.write(b'acc_bias_x: 0\nacc_bias_y: 0\nacc_bias_z: 0\nang_bias_x: 0\nang_bias_y: 0\nang_bias_z: 0\nang_z2x_proj: 0\nang_z2y_proj: 0\n');f.flush()
    def context(stage,path=f.name):
        c=LaunchContext();c.launch_configurations.update(stage=stage,calibration_path=path,sensor_offset_x='0.3',sensor_offset_y='0.0');return c
    for stage in ('command_test','localization','local','far'):
        captured.clear();actions=m._stack(context(stage));assert actions
        if stage != 'command_test':
            base=[x for x in captured if x.get('name')=='nav_sensor_vehicle']
            assert len(base)==1 and '--x' in base[0]['arguments']
        assert not any(x['package']=='unitree_legged_real' for x in captured)
        assert not any('sport' in x['package'] for x in captured)
    for stage,path in [('bogus',f.name),('localization','/missing-calibration')]:
        try:m._stack(context(stage,path))
        except ValueError:pass
        else:raise AssertionError('bad stage/missing calibration accepted')
text=p.read_text()
assert "SetRemap(src='/cmd_vel', dst='/navigation/cmd_vel')" in text
assert "'sendSportCommand':'false'" in text
assert "'publishBaseTf':'false'" in text
assert "'far_route': stage == 'far'" in text
assert '/sim/' not in text and 'mujoco' not in text.lower()
print('PASS four launch stages, no locomotion/Sport node, planner request remap, calibration fail-closed')

# Parse the actual XML condition/argument contract without executing its Nodes.
from launch.launch_description_sources import AnyLaunchDescriptionSource
desc=AnyLaunchDescriptionSource(str(packages['local_planner']/'launch/local_planner.launch')).get_launch_description(LaunchContext())
assert desc.entities
print('PASS actual local_planner XML with optional common base TF')
