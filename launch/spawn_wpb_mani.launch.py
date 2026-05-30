#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _continue_on_success(success_actions, process_label, shutdown_on_failure=True):
    def _handler(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg=f"[wpr_simulation2] {process_label} completed successfully."),
                *success_actions,
            ]

        failure_actions = [
            LogInfo(
                msg=(
                    f"[wpr_simulation2] {process_label} failed with exit code "
                    f"{event.returncode}. Stopping remaining startup actions."
                )
            ),
        ]
        if shutdown_on_failure:
            failure_actions.append(Shutdown(reason=f"{process_label} failed"))

        return failure_actions

    return _handler


def generate_launch_description():
    pose_x = LaunchConfiguration("pose_x")
    pose_y = LaunchConfiguration("pose_y")
    pose_theta = LaunchConfiguration("pose_theta")
    spawn_delay = LaunchConfiguration("spawn_delay")
    controller_delay = LaunchConfiguration("controller_delay")
    manipulator_delay = LaunchConfiguration("manipulator_delay")
    controller_params = PathJoinSubstitution(
        [FindPackageShare("wpr_simulation2"), "config", "wpb_home_controller.yaml"]
    )
    helper_script = PathJoinSubstitution(
        [FindPackagePrefix("wpr_simulation2"), "lib", "wpr_simulation2", "ensure_controller_active.py"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "models", "wpb_home_mani.model"]
            ),
            " ",
            "controller_config:=",
            controller_params,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/kinect2/qhd/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/kinect2/sd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/scan", "/scan_bridge"),
            ("/imu/data", "/imu/data_bridge"),
            ("/kinect2/qhd/image_raw", "/kinect2/qhd/image_raw_bridge"),
            ("/kinect2/sd/points", "/kinect2/sd/points_bridge"),
        ],
        output="screen",
    )

    sensor_frame_normalizer = Node(
        package="wpr_simulation2",
        executable="sensor_frame_normalizer.py",
        name="sensor_frame_normalizer",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "topic": "robot_description",
                "name": "wpb_home_mani",
                "allow_renaming": False,
                "x": pose_x,
                "y": pose_y,
                "z": 0.0,
                "Y": pose_theta,
            }
        ],
    )

    planar_move = Node(
        package="wpr_simulation2",
        executable="gz_planar_move.py",
        name="wpb_home_planar_move",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "entity_name": "wpb_home_mani",
                "world_name": "default",
                "initial_x": pose_x,
                "initial_y": pose_y,
                "initial_yaw": pose_theta,
                "initial_z": 0.0,
            }
        ],
    )

    joint_state_broadcaster_ensure = ExecuteProcess(
        cmd=[
            helper_script,
            "--controller-name",
            "joint_state_broadcaster",
            "--timeout-sec",
            "60.0",
        ],
        output="screen",
    )

    manipulator_controller_ensure = ExecuteProcess(
        cmd=[
            helper_script,
            "--controller-name",
            "manipulator_controller",
            "--timeout-sec",
            "60.0",
        ],
        output="screen",
    )

    wpb_home_mani_sim = Node(
        package="wpr_simulation2",
        executable="wpb_home_mani_sim",
        name="wpb_home_mani_sim",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_x", default_value="0.0"),
            DeclareLaunchArgument("pose_y", default_value="0.0"),
            DeclareLaunchArgument("pose_theta", default_value="0.0"),
            DeclareLaunchArgument("spawn_delay", default_value="3.0"),
            DeclareLaunchArgument("controller_delay", default_value="2.0"),
            DeclareLaunchArgument("manipulator_delay", default_value="5.0"),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=_continue_on_success(
                        [
                            planar_move,
                            TimerAction(
                                period=controller_delay,
                                actions=[joint_state_broadcaster_ensure],
                            ),
                            TimerAction(
                                period=manipulator_delay,
                                actions=[manipulator_controller_ensure],
                            ),
                        ],
                        "robot spawn",
                    ),
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=manipulator_controller_ensure,
                    on_exit=_continue_on_success(
                        [wpb_home_mani_sim],
                        "manipulator controller activation",
                        shutdown_on_failure=False,
                    ),
                )
            ),
            bridge,
            sensor_frame_normalizer,
            robot_state_publisher,
            # Give Gazebo a short warm-up window so model creation doesn't race the world startup.
            TimerAction(period=spawn_delay, actions=[spawn_robot]),
        ]
    )
