#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class GzPlanarMove(Node):
    def __init__(self) -> None:
        super().__init__("gz_planar_move")

        self.declare_parameter("entity_name", "wpb_home_mani")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("set_pose_request_timeout", 0.25)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.entity_name = self.get_parameter("entity_name").value
        self.world_name = self.get_parameter("world_name").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom_enabled = self.get_parameter("publish_odom").value
        self.publish_odom_tf_enabled = self.get_parameter("publish_odom_tf").value
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.cmd_vel_timeout = Duration(
            seconds=float(self.get_parameter("cmd_vel_timeout").value)
        )
        self.set_pose_request_timeout = Duration(
            seconds=float(self.get_parameter("set_pose_request_timeout").value)
        )

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.pending_request = None
        self.pending_request_started_at = None
        self.service_name = f"/world/{self.world_name}/set_pose"
        self.service_wait_log_period = Duration(seconds=2.0)
        self.last_service_wait_log_time = self.get_clock().now()
        self.received_motion_command = False
        self.pose_service_ready_logged = False
        self.pose_command_success_logged = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.set_pose_client = self.create_client(SetEntityPose, self.service_name)
        self.create_timer(1.0 / self.update_rate, self.on_timer)
        self.get_logger().info(
            "Planar move bridge ready for entity "
            f"'{self.entity_name}' on '{self.cmd_vel_topic}', "
            f"driving Gazebo service '{self.service_name}'."
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        if not self.received_motion_command and self._is_motion_command(msg):
            self.received_motion_command = True
            self.get_logger().info(
                "Received first non-zero /cmd_vel command; planar movement is armed."
            )

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        self.last_update_time = now
        cmd = Twist()
        if now - self.last_cmd_time <= self.cmd_vel_timeout:
            cmd = self.cmd_vel

        if self.pending_request is not None and not self.pending_request.done():
            if self._pending_request_is_fresh(now):
                self.publish_odom_message(now, cmd)
                return
            # Why: a single stalled ros_gz service response must not freeze
            # odom/TF forever; Nav2 otherwise keeps commanding motion and aborts
            # with "Failed to make progress" while the simulated robot is stuck.
            self.get_logger().warning(
                "Gazebo set_pose request timed out; dropping the stale request "
                "and continuing planar odometry integration."
            )
            self.pending_request = None
            self.pending_request_started_at = None

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        world_vx = cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw
        world_vy = cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw

        next_x = self.x + world_vx * dt
        next_y = self.y + world_vy * dt
        next_yaw = self.yaw + cmd.angular.z * dt

        if self.send_pose_request(next_x, next_y, next_yaw, now, cmd):
            self.x = next_x
            self.y = next_y
            self.yaw = next_yaw
        self.publish_odom_message(now, cmd)

    def send_pose_request(
        self, next_x: float, next_y: float, next_yaw: float, now, cmd: Twist
    ) -> bool:
        if not self.set_pose_client.service_is_ready():
            if self._is_motion_command(cmd) and now - self.last_service_wait_log_time >= self.service_wait_log_period:
                self.get_logger().warning(
                    "Gazebo set_pose service is not ready yet. "
                    f"Expected service: '{self.service_name}'."
                )
                self.last_service_wait_log_time = now
            return False

        if not self.pose_service_ready_logged:
            self.pose_service_ready_logged = True
            self.get_logger().info(
                f"Connected to Gazebo pose service '{self.service_name}'."
            )

        request = SetEntityPose.Request()
        request.entity = Entity(name=self.entity_name, type=Entity.MODEL)
        request.pose.position.x = next_x
        request.pose.position.y = next_y
        request.pose.position.z = self.z
        (
            request.pose.orientation.x,
            request.pose.orientation.y,
            request.pose.orientation.z,
            request.pose.orientation.w,
        ) = quaternion_from_yaw(next_yaw)

        self.pending_request = self.set_pose_client.call_async(request)
        self.pending_request_started_at = now
        self.pending_request.add_done_callback(self.on_set_pose_response)
        return True

    def on_set_pose_response(self, future) -> None:
        if future is not self.pending_request:
            return
        self.pending_request = None
        self.pending_request_started_at = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime safeguard
            self.get_logger().warning(f"Gazebo set_pose request raised an exception: {exc}")
            return

        if not response.success:
            self.get_logger().warning(
                f"Gazebo rejected a set_pose request for entity '{self.entity_name}'."
            )
            return

        if not self.pose_command_success_logged:
            self.pose_command_success_logged = True
            self.get_logger().info(
                f"Gazebo confirmed pose updates for entity '{self.entity_name}'."
            )

    def publish_odom_message(self, now, cmd: Twist) -> None:
        quat = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist = cmd
        if self.publish_odom_enabled:
            self.odom_pub.publish(odom)

        if not self.publish_odom_tf_enabled:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)

    def _pending_request_is_fresh(self, now) -> bool:
        if self.pending_request_started_at is None:
            return False
        return now - self.pending_request_started_at <= self.set_pose_request_timeout

    @staticmethod
    def _is_motion_command(cmd: Twist) -> bool:
        return any(
            abs(value) > 1e-6
            for value in (
                cmd.linear.x,
                cmd.linear.y,
                cmd.linear.z,
                cmd.angular.x,
                cmd.angular.y,
                cmd.angular.z,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzPlanarMove()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
