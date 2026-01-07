#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class SendJointTrajectory(Node):
    def __init__(self):
        super().__init__("send_joint_trajectory")

        # Parameters
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("target_joints", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("wait_result", True)

        controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
        target = self.get_parameter("target_joints").get_parameter_value().double_array_value
        duration_sec = float(self.get_parameter("duration_sec").value)
        self.wait_result = bool(self.get_parameter("wait_result").value)

        if len(self.joint_names) != 6:
            raise RuntimeError(f"joint_names must have 6 elements, got {len(self.joint_names)}")
        if len(target) != 6:
            raise RuntimeError(f"target_joints must have 6 elements, got {len(target)}")

        action_name = f"/{controller_name}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

        self.get_logger().info(f"Waiting for action server: {action_name}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server not available: {action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in target]
        p.time_from_start.sec = int(duration_sec)
        p.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [p]

        self.get_logger().info(
            "Sending trajectory:\n"
            f"  joints: {goal.trajectory.joint_names}\n"
            f"  target: {p.positions}\n"
            f"  duration: {duration_sec:.3f}s"
        )

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        if not self.wait_result:
            self.get_logger().info("wait_result:=false -> exiting.")
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Result received. status={status}, error_code={result.error_code}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SendJointTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
