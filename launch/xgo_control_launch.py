import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('xgo2_ros'),
        'urdf',
        'mini2_description.urdf'
    )
    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    params = {'robot_description': robot_desc}

    return LaunchDescription([
        DeclareLaunchArgument(
            'com_port',
            default_value='/dev/ttyS0',
            description='COM port argument'
        ),
        DeclareLaunchArgument(
            'is_xgomini',
            default_value='true',
            description='Is XGOMini argument'
        ),
        DeclareLaunchArgument(
            'enable_joint_gui',
            default_value='false',
            description='Enable joint GUI argument'
        ),

        Node(
            package='xgo2_ros',
            executable='xgo2_ros_node',
            name='xgo2_ros_node',
            output='screen',
            parameters=[
                {'com_port': LaunchConfiguration('com_port')},
                {'enable_joint_gui': LaunchConfiguration('enable_joint_gui')}
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        )
    ])
