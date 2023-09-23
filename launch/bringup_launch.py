#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    description_package = FindPackageShare('xgo2_ros')
    description_path = PathJoinSubstitution(
        [description_package, 'urdf', 'mini2_description.urdf']
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, 'config', 'joints.yaml']
    )
    links_config_path = PathJoinSubstitution(
        [description_package, 'config', 'links.yaml']
    )
    gait_config_path = PathJoinSubstitution(
        [description_package, 'config', 'gait.yaml']
    )
    
    servo_interface_launch_path = PathJoinSubstitution(
        [description_package, 'launch', 'xgo_control_launch.py']
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_description_path = DeclareLaunchArgument(
        name="description_path",
        default_value=description_path,
        description="Absolute path to robot urdf file",
    )

    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path",
        default_value=joints_config_path,
        description="Absolute path to joints map file",
    )

    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path",
        default_value=links_config_path,
        description="Absolute path to links map file",
    )

    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path",
        default_value=gait_config_path,
        description="Absolute path to gait config file",
    )

    declare_orientation_from_imu = DeclareLaunchArgument(
        "orientation_from_imu", default_value="false", description="Take orientation from IMU data"
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )

    declare_rviz_ref_frame = DeclareLaunchArgument(
        "rviz_ref_frame", default_value="odom", description="Rviz ref frame"
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="", description="Robot name"
    )

    declare_base_link_frame = DeclareLaunchArgument(
        "base_link_frame", default_value="base_link", description="Base link frame"
    )

    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )

    declare_gazebo = DeclareLaunchArgument(
        "gazebo", default_value="false", description="If in gazebo"
    )

    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic",
        default_value="joint_group_effort_controller/joint_trajectory",
        description="Joint controller topic",
    )

    declare_hardware_connected = DeclareLaunchArgument(
        "joint_hardware_connected",
        default_value="true",
        description="Whether hardware is connected",
    )

    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control",
        default_value="true",
        description="Publish joint control",
    )

    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states",
        default_value="false",
        description="Publish joint states",
    )

    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts",
        default_value="true",
        description="Publish foot contacts",
    )

    declare_publish_odom_tf = DeclareLaunchArgument(
        "publish_odom_tf",
        default_value="true",
        description="Publish odom tf from cmd_vel estimation",
    )

    declare_close_loop_odom = DeclareLaunchArgument(
        "close_loop_odom", default_value="true", description=""
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": ParameterValue(Command(['cat ', LaunchConfiguration('description_path')]), value_type=str)},
            LaunchConfiguration('joints_map_path'),
            LaunchConfiguration('links_map_path'),
            LaunchConfiguration('gait_config_path'),
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"orientation_from_imu": LaunchConfiguration("orientation_from_imu")},
            {"urdf": ParameterValue(Command(['cat ', LaunchConfiguration('description_path')]), value_type=str)},
            LaunchConfiguration('joints_map_path'),
            LaunchConfiguration('links_map_path'),
            LaunchConfiguration('gait_config_path'),
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_path")],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    
    servo_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_interface_launch_path)
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            declare_joints_map_path,
            declare_links_map_path,
            declare_gait_config_path,
            declare_orientation_from_imu,
            declare_rviz,
            declare_rviz_ref_frame,
            declare_robot_name,
            declare_base_link_frame,
            declare_lite,
            declare_gazebo,
            declare_joint_controller_topic,
            declare_hardware_connected,
            declare_publish_joint_control,
            declare_publish_joint_states,
            declare_publish_foot_contacts,
            declare_publish_odom_tf,
            declare_close_loop_odom,
            servo_interface_launch,
            quadruped_controller_node,
            state_estimator_node,
            base_to_footprint_ekf,
            footprint_to_odom_ekf,
            rviz2
        ]
    )