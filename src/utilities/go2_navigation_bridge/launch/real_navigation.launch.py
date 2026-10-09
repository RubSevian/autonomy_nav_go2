"""Sensor/planner side only. Existing R3 launch is started separately by operator."""
from pathlib import Path
import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

def _stack(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    stage = get('stage')
    if stage not in ('command_test', 'localization', 'local', 'far'):
        raise ValueError('Unknown navigation stage: ' + stage)
    actions = []
    if stage == 'command_test':
        return [Node(package='go2_navigation_bridge', executable='go2_navigation_bridge.py', parameters=[{'input_mode': 'command_test'}], output='screen')]
    calibration = Path(get('calibration_path')).expanduser()
    if not calibration.is_file():
        raise ValueError('Real IMU calibration file required: ' + str(calibration))
    values = yaml.safe_load(calibration.read_text())
    keys = ('acc_bias_x','acc_bias_y','acc_bias_z','ang_bias_x','ang_bias_y','ang_bias_z','ang_z2x_proj','ang_z2y_proj')
    if not isinstance(values, dict) or any(key not in values or not isinstance(values[key], (int,float)) or not math.isfinite(values[key]) for key in keys):
        raise ValueError('Calibration must contain eight finite bias/projection values')
    def include(package, filename, args=None):
        path = str(Path(get_package_share_directory(package)) / 'launch' / filename)
        return IncludeLaunchDescription(AnyLaunchDescriptionSource(path), launch_arguments=(args or {}).items())
    actions.append(include('point_lio_unilidar', 'mapping_utlidar.launch', {'rviz': 'false', 'calibration_path': str(calibration)}))
    # Preserve Jazzy frame relationships; no simulated sensor/GT nodes.
    actions += [Node(package='tf2_ros', executable='static_transform_publisher', name='nav_map_camera_init', arguments=['--frame-id','map','--child-frame-id','camera_init']),
                Node(package='tf2_ros', executable='static_transform_publisher', name='nav_aft_sensor', arguments=['--frame-id','aft_mapped','--child-frame-id','sensor'])]
    actions.append(Node(package='go2_navigation_bridge', executable='go2_navigation_bridge.py', parameters=[{'input_mode': 'stack', 'local_route': stage == 'local'}], output='screen'))
    if stage in ('local','far'):
        # Remap planner output only; bridge is the sole publisher on physical /cmd_vel.
        planners = [SetRemap(src='/cmd_vel', dst='/navigation/cmd_vel'),
                    include('local_planner','local_planner.launch', {'autonomyMode':'true','maxSpeed':'0.2','autonomySpeed':'0.2','is_real_robot':'false','sendSportCommand':'false','sensorOffsetX':get('sensor_offset_x'),'sensorOffsetY':get('sensor_offset_y')}),
                    include('terrain_analysis','terrain_analysis.launch'),
                    include('terrain_analysis_ext','terrain_analysis_ext.launch')]
        if stage == 'far':
            planners.append(include('far_planner','far_planner.launch.py',{'rviz':'false'}))
        actions.append(GroupAction(actions=planners))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage',default_value='localization'),
        DeclareLaunchArgument('calibration_path',default_value=str(Path.home()/'Desktop/imu_calib_data.yaml')),
        DeclareLaunchArgument('sensor_offset_x',default_value='0.3'),
        DeclareLaunchArgument('sensor_offset_y',default_value='0.0'),
        OpaqueFunction(function=_stack)])
