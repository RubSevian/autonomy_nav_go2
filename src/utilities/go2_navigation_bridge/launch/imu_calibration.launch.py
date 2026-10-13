"""Physical Sport calibration only; operator confirms stock Sport and free space."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _calibration(context):
    share = Path(get_package_share_directory('go2_navigation_bridge'))
    spec = importlib.util.spec_from_file_location('navigation_profile', share/'scripts/navigation_profile.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profile = module.load_profile(LaunchConfiguration('profile_path').perform(context))
    return [Node(package='calibrate_imu', executable='calibrate_imu', name='calibrate_imu', output='screen',
                 parameters=[{'rotation_speed_rps':float(profile['calibration']['rotation_speed_rps']), 'use_sim_time':False}],
                 remappings=[('/utlidar/imu', profile['sensors']['raw_imu_topic'])])]


def generate_launch_description():
    share = Path(get_package_share_directory('go2_navigation_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('profile_path', default_value=str(share/'config/real_navigation.yaml')),
        OpaqueFunction(function=_calibration)])
