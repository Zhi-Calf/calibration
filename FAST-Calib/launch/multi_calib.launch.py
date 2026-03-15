import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_calib')
    config = os.path.join(pkg_share, 'config', 'qr_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2'),

        Node(
            package='fast_calib',
            executable='multi_fast_calib',
            name='multi_fast_calib',
            parameters=[config],
            output='screen',
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
