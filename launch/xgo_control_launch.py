from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root_pkg = get_package_share_directory('xgo2_ros')
    config_path = path.join(root_pkg, 'config', 'config.json')
    use_lidar = LaunchConfiguration('use_lidar', default=False)

    return LaunchDescription([
        Node(
            package='xgo2_ros',
            executable='xgo2_ros_node',
            name='xgo2_ros_node',
            output='screen',
            parameters=[{'config_path': config_path}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                path.join(get_package_share_directory('ldlidar_node'), 'launch', 'ldlidar_with_mgr.launch.py')
            ),
            condition=IfCondition(use_lidar)
        )
    ])
