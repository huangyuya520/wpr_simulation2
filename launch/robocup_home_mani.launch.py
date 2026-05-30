#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Open a visible Gazebo Sim GUI window.",
    )
    gazebo_gui_gz_version_arg = DeclareLaunchArgument(
        "gazebo_gui_gz_version",
        default_value="8",
        description="Gazebo Sim major version passed to the GUI-only client.",
    )
    spawn_objects_arg = DeclareLaunchArgument(
        "spawn_objects",
        default_value="true",
        description="Spawn furniture and drink objects.",
    )
    spawn_persons_arg = DeclareLaunchArgument(
        "spawn_persons",
        default_value="true",
        description="Spawn guest/person models.",
    )
    scene_delay_arg = DeclareLaunchArgument(
        "scene_delay",
        default_value="20.0",
        description="Delay full scene object spawning until Gazebo and robot spawn settle.",
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
            "gazebo_gui_gz_version": LaunchConfiguration("gazebo_gui_gz_version"),
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_mani.launch.py"]
            )
        ),
        launch_arguments={"pose_x": "-4.0", "pose_y": "-0.5", "pose_theta": "0.0"}.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_objects.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("spawn_objects")),
    )

    spawn_persons = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_persons.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("spawn_persons")),
    )

    return LaunchDescription(
        [
            gazebo_gui_arg,
            gazebo_gui_gz_version_arg,
            spawn_objects_arg,
            spawn_persons_arg,
            scene_delay_arg,
            world,
            spawn_robot,
            TimerAction(period=LaunchConfiguration("scene_delay"), actions=[spawn_objects]),
            TimerAction(period=LaunchConfiguration("scene_delay"), actions=[spawn_persons]),
        ]
    )
