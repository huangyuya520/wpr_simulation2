#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gazebo_gui_gz_version = LaunchConfiguration("gazebo_gui_gz_version")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments=[("gz_args", ["-s -r -v 1 ", world])],
    )

    gazebo_gui_cmd = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-g",
            "-v",
            "1",
        ],
        name="gazebo_gui",
        output="screen",
        additional_env={
            "QT_X11_NO_MITSHM": "1",
            "QT_QPA_PLATFORM": "xcb",
            "MESA_GL_VERSION_OVERRIDE": "3.3",
            "OGRE_RTT_MODE": "Copy",
        },
        condition=IfCondition(gazebo_gui),
    )

    gazebo_window_raise_cmd = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("wpr_simulation2"),
                    "lib",
                    "wpr_simulation2",
                    "raise_gazebo_window.py",
                ]
            ),
            "--title",
            "Gazebo Sim",
            "--x",
            "80",
            "--y",
            "40",
            "--width",
            "1200",
            "--height",
            "850",
            "--timeout",
            "20",
        ],
        name="gazebo_window_raise",
        output="screen",
        condition=IfCondition(gazebo_gui),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    set_pose_bridge = RosGzBridge(
        bridge_name="gazebo_service_bridge",
        extra_bridge_params={
            "bridges": {
                "set_pose_bridge": {
                    "service_name": "/world/default/set_pose",
                    "ros_type_name": "ros_gz_interfaces/srv/SetEntityPose",
                    "gz_req_type_name": "gz.msgs.Pose",
                    "gz_rep_type_name": "gz.msgs.Boolean",
                }
            },
            "bridge_names": ["set_pose_bridge"],
        },
        log_level="warn",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wpr_simulation2"), "worlds", "robocup_home.world"]
                ),
                description="Absolute path to the Gazebo Sim world file",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Open a visible Gazebo Sim GUI window.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui_gz_version",
                default_value="8",
                description=(
                    "Deprecated compatibility argument. The GUI client now lets gz "
                    "auto-select the installed Gazebo Sim version."
                ),
            ),
            gz_sim,
            # Why: launch tracks the GUI client directly; wrapping it in xterm can
            # leave WSLg showing a terminal thumbnail instead of a usable Gazebo window.
            TimerAction(period=1.0, actions=[gazebo_gui_cmd]),
            TimerAction(period=3.0, actions=[gazebo_window_raise_cmd]),
            clock_bridge,
            set_pose_bridge,
        ]
    )
