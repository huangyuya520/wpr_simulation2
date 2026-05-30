#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawn_objects_arg = DeclareLaunchArgument(
        "spawn_objects",
        default_value="true",
        description="Spawn furniture and small objects after the robot is inserted.",
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Open a visible Gazebo Sim GUI window.",
    )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "world.launch.py"]
            )
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "worlds", "robocup_home.world"]
            ),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_lidar.launch.py"]
            )
        ),
        launch_arguments={"pose_x": "-6.0", "pose_y": "-0.5", "pose_theta": "0.0"}.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_objects.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("spawn_objects")),
    )

    return LaunchDescription(
        [
            spawn_objects_arg,
            gazebo_gui_arg,
            world,
            spawn_robot,
            spawn_objects,
        ]
    )
