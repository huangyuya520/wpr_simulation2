#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawn_objects_arg = DeclareLaunchArgument(
        "spawn_objects",
        default_value="false",
        description="Set true to spawn the complete furniture/object scene during SLAM.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Start RViz with the SLAM view.",
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Open a visible Gazebo Sim GUI window.",
    )
    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("wpr_simulation2"),
                "config",
                "slam_toolbox_mapping.yaml",
            ]
        ),
        description="Full path to the slam_toolbox mapping parameters file.",
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "robocup_home.launch.py"]
            )
        ),
        launch_arguments={
            "spawn_objects": LaunchConfiguration("spawn_objects"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_sync_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "slam_params_file": LaunchConfiguration("slam_params_file"),
        }.items(),
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "rviz", "slam.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            spawn_objects_arg,
            launch_rviz_arg,
            gazebo_gui_arg,
            slam_params_file_arg,
            gazebo_cmd,
            slam_cmd,
            # Why: RViz's slam_toolbox panel queries lifecycle parameters on startup;
            # a short delay avoids a noisy race while Gazebo and SLAM are still configuring.
            TimerAction(period=3.0, actions=[rviz_cmd]),
        ]
    )
