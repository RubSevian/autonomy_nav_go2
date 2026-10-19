"""Sensor/planner side only. Existing R3 launch is started separately by operator."""
from pathlib import Path
import math
import yaml
import importlib.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _stack(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    stage = get('stage')
    if stage not in ('command_test', 'localization', 'local', 'far'):
        raise ValueError('Unknown navigation stage: ' + stage)
    share = Path(get_package_share_directory('go2_navigation_bridge'))
    spec = importlib.util.spec_from_file_location('navigation_profile', share / 'scripts' / 'navigation_profile.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profile = module.load_profile(get('profile_path'))
    # An explicit launch offset overrides the common value before any node/TF is created.
    for arg,key in (('sensor_offset_x','sensor_offset_x_m'), ('sensor_offset_y','sensor_offset_y_m')):
        if get(arg):
            value = float(get(arg))
            if not math.isfinite(value):
                raise ValueError('Sensor offsets must be finite')
            profile['robot'][key] = value
    robot, local, bridge, sensors = (profile[name] for name in ('robot','local','bridge','sensors'))
    parameters = module.component_parameters(profile)
    bridge_params = dict({key:(value if key in ('tf_pair_max_samples', 'odometry_subscription_depth') else float(value)) for key,value in bridge.items()}, goal_tolerance_m=float(local['goal_tolerance_m']),
                         imu_topic=sensors['raw_imu_output_topic'], lidar_topic=sensors['cloud_topic'], use_sim_time=False)
    observe_only = get('observe_only').lower()
    if observe_only not in ('true', 'false'):
        raise ValueError('observe_only must be true or false')
    observe_only = observe_only == 'true'
    if stage != 'command_test' and sensors.get('time_alignment_variant') != 'bounded_common_v1':
        raise ValueError('Normal NAV requires bounded_common_v1 clock mapping')
    bridge_params['require_clock_alignment'] = stage != 'command_test'
    # Sensors/planners remain on their normal topics. Both control outputs of
    # the adapter are disconnected from the leg controller in manual sessions.
    bridge_remaps = [('/cmd_vel', '/diagnostics/manual_nav/cmd_vel'),
                     ('/go2/mission/readiness', '/diagnostics/manual_nav/readiness')] if observe_only else []
    actions = []
    if stage == 'command_test':
        return [Node(package='go2_navigation_bridge', executable='go2_navigation_bridge.py', parameters=[dict(bridge_params, input_mode='command_test')], output='screen')]
    calibration = Path(get('calibration_path')).expanduser()
    if not calibration.is_file():
        raise ValueError('Real IMU calibration file required: ' + str(calibration))
    values = yaml.safe_load(calibration.read_text())
    keys = ('acc_bias_x','acc_bias_y','acc_bias_z','ang_bias_x','ang_bias_y','ang_bias_z','ang_z2x_proj','ang_z2y_proj')
    if not isinstance(values, dict) or any(key not in values or type(values[key]) not in (int,float) or not math.isfinite(values[key]) for key in keys):
        raise ValueError('Calibration must contain eight finite bias/projection values')
    # Direct nodes intentionally avoid hidden overrides in legacy XML/YAML files.
    actions.append(Node(package='transform_sensors', executable='transform_everything', name='transform_everything',
                        parameters=[dict(sensors, calibration_path=str(calibration), use_sim_time=False)], output='screen'))
    actions.append(Node(package='point_lio_unilidar', executable='pointlio_mapping', name='laserMapping',
                        parameters=[parameters['laserMapping']], output='screen',
                        remappings=[('/path','/point_lio/path'),('/cloud_registered','/registered_scan'),('/aft_mapped_to_init','/state_estimation')]))
    # Preserve Jazzy frame relationships; no simulated sensor/GT nodes.
    offsets = [float(robot[key]) for key in ('sensor_offset_x_m','sensor_offset_y_m')]
    actions += [Node(package='tf2_ros', executable='static_transform_publisher', name='nav_map_camera_init', arguments=['--frame-id','map','--child-frame-id','camera_init']),
                Node(package='tf2_ros', executable='static_transform_publisher', name='nav_aft_sensor', arguments=['--frame-id','aft_mapped','--child-frame-id','sensor']),
                Node(package='tf2_ros', executable='static_transform_publisher', name='nav_sensor_vehicle', arguments=['--frame-id','sensor','--child-frame-id','vehicle','--x',str(-offsets[0]),'--y',str(-offsets[1])])]
    actions.append(Node(package='go2_navigation_bridge', executable='go2_navigation_bridge.py', parameters=[dict(bridge_params, input_mode='stack', local_route=stage == 'local', far_route=stage == 'far')], remappings=bridge_remaps, output='screen'))
    actions.append(Node(package='rviz2', executable='rviz2', name='real_nav_rviz',
                        arguments=['-d',str(share/'config/real_navigation.rviz')],
                        remappings=[('/goal_point', '/way_point' if stage == 'local' else '/goal_point')],
                        condition=IfCondition(LaunchConfiguration('rviz')), output='screen'))
    if stage in ('local','far'):
        # Remap planner output only; bridge publishes /cmd_vel diagnostics and the atomic R3 envelope.
        parameters['localPlanner']['pathFolder'] = str(Path(get_package_share_directory('local_planner'))/'paths')
        for package, executable in (('local_planner','localPlanner'), ('local_planner','pathFollower'),
                                    ('terrain_analysis','terrainAnalysis'), ('terrain_analysis_ext','terrainAnalysisExt')):
            actions.append(Node(package=package, executable=executable, name=executable, parameters=[parameters[executable]],
                                remappings=[('/cmd_vel','/navigation/cmd_vel')] if executable == 'pathFollower' else [], output='screen'))
        if stage == 'far':
            actions.append(Node(package='far_planner', executable='far_planner', name='far_planner', parameters=[parameters['far_planner']],
                                remappings=[('/odom_world','/state_estimation'),('/terrain_cloud','/terrain_map_ext'),('/scan_cloud','/terrain_map'),('/terrain_local_cloud','/registered_scan')], output='screen'))
            actions.append(Node(package='graph_decoder', executable='graph_decoder', name='graph_decoder', parameters=[parameters['graph_decoder']], output='screen'))
    if stage == 'far' and profile['recording']['enabled']:
        recording = profile['recording']
        actions.append(Node(package='go2_navigation_bridge', executable='nav_drift.py', name='manual_drift_recorder',
                            arguments=['--config', get('profile_path')], output='screen',
                            sigterm_timeout=str(recording['shutdown_timeout_s']), sigkill_timeout=str(recording['shutdown_timeout_s'])))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage',default_value='localization'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('observe_only',default_value='false'),
        DeclareLaunchArgument('calibration_path',default_value=str(Path.home()/'Desktop/imu_calib_data.yaml')),
        DeclareLaunchArgument('profile_path', default_value=str(Path(get_package_share_directory('go2_navigation_bridge'))/'config/real_navigation.yaml')),
        DeclareLaunchArgument('sensor_offset_x',default_value=''),
        DeclareLaunchArgument('sensor_offset_y',default_value=''),
        OpaqueFunction(function=_stack)])
