from os import path
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root_pkg = get_package_share_directory('xgo2_ros')
    config_path = path.join(root_pkg, 'config', 'config.json')
    urdf_path = path.join(root_pkg, 'urdf', 'mini2_description.urdf')
    
    def read_urdf():
        with open(urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()
        return {'robot_description': robot_description}
    
    use_lidar = LaunchConfiguration('use_lidar', default=False)
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher', default=False)

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
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[read_urdf()],
            condition=IfCondition(use_robot_state_publisher)
        )
    ])
