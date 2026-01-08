#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class SendJointTrajectory(Node):
    def __init__(self):
        super().__init__("send_joint_trajectory")

        # Example parameters (you already have your YAML)
        self.declare_parameter("joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("target", [0.0, -0.8, 1.2, 0.0, 0.6, 0.0])
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")

        self._action_name = self.get_parameter("action_name").value
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    def send_once_and_exit(self):
        joints = self.get_parameter("joints").value
        target = self.get_parameter("target").value
        duration = float(self.get_parameter("duration").value)

        self.get_logger().info(f"Waiting for action server: {self._action_name}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available.")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)

        p = JointTrajectoryPoint()
        p.positions = list(target)
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [p]

        self.get_logger().info("Sending trajectory:")
        self.get_logger().info(f"  joints: {goal.trajectory.joint_names}")
        self.get_logger().info(f"  target: {p.positions}")
        self.get_logger().info(f"  duration: {duration:.3f}s")

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
            f"Result received. status={result.status}, error_code={result.result.error_code}"
        )
        return 0


def main():
    rclpy.init()
    node = SendJointTrajectory()

    try:
        rc = node.send_once_and_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():   # prevents: "Context must be initialized..."
            rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == "__main__":
    main()
