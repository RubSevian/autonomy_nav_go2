import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value='false'),
        DeclareLaunchArgument('config', default_value='default'),
        DeclareLaunchArgument('rviz', default_value='false'),

        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            ros_arguments=['--log-level', 'rmw_cyclonedds_cpp:=error'],
            output='screen',
            parameters=[
                PythonExpression([
                '"', 
                get_package_share_directory('far_planner'), 
                '/config/', 
                LaunchConfiguration('config'), 
                '.yaml"'])
            ],
            remappings=[
                ('/odom_world', '/state_estimation'),
                ('/terrain_cloud', '/terrain_map_ext'),
                ('/scan_cloud', '/terrain_map'),
                ('/terrain_local_cloud', '/registered_scan')
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='far_rviz',
            ros_arguments=['--log-level', 'rmw_cyclonedds_cpp:=error'],
            arguments=['-d', 
                PythonExpression([
                '"', 
                get_package_share_directory('far_planner'), 
                '/rviz/', 
                LaunchConfiguration('config'), 
                '.rviz"'])
            ],
            respawn=False,
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Including another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('graph_decoder'), '/launch/decoder.launch.py'])
        )
    ])
