#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quat_to_rpy(x: float, y: float, z: float, w: float):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SendJointTrajectory(Node):
    def __init__(self):
        super().__init__("send_joint_trajectory")

        # Parameters (YAML IN DEGREES)
        self.declare_parameter(
            "joints",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "target",   # degrees
            [0.0, -45.0, 60.0, 0.0, 30.0, 0.0],
        )
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")

        # TF pose query
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "link6")
        self.declare_parameter("tf_timeout", 2.0)

        self._action_name = self.get_parameter("action_name").value
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

        # TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _log_tip_pose(self) -> bool:
        base = self.get_parameter("base_frame").value
        tip = self.get_parameter("tip_frame").value
        timeout = float(self.get_parameter("tf_timeout").value)

        try:
            tf = self._tf_buffer.lookup_transform(
                base,
                tip,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed for {base} -> {tip}: {e}")
            return False

        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        self.get_logger().info(f"End-effector pose (TF): {base} -> {tip}")
        self.get_logger().info(
            f"  position [m]: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}"
        )
        self.get_logger().info(
            f"  rpy [deg]   : "
            f"roll={math.degrees(roll):.2f}, "
            f"pitch={math.degrees(pitch):.2f}, "
            f"yaw={math.degrees(yaw):.2f}"
        )
        return True

    def send_once_and_exit(self):
        joints = self.get_parameter("joints").value
        target_deg = self.get_parameter("target").value
        duration = float(self.get_parameter("duration").value)

        # Convert DEGREES -> RADIANS (internal, silent)
        target_rad = [math.radians(float(v)) for v in target_deg]

        self.get_logger().info(f"Waiting for action server: {self._action_name}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available.")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)

        p = JointTrajectoryPoint()
        p.positions = target_rad
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [p]

        self.get_logger().info("Sending trajectory:")
        self.get_logger().info(f"  joints        : {goal.trajectory.joint_names}")
        self.get_logger().info(f"  target [deg]  : {[float(v) for v in target_deg]}")
        self.get_logger().info(f"  duration [s]  : {duration:.3f}")

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return 2

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error("No result returned.")
            return 3

        self.get_logger().info(
            f"Result received. status={result.status}, "
            f"error_code={result.result.error_code}"
        )

        if not self._log_tip_pose():
            self.get_logger().warn(
                "Could not read end-effector TF. "
                "Check robot_state_publisher and joint_state_broadcaster."
            )

        return 0


def main():
    rclpy.init()
    node = SendJointTrajectory()
    try:
        rc = node.send_once_and_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
