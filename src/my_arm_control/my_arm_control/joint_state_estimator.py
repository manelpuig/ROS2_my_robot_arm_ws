#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateEstimator(Node):
    def __init__(self):
        super().__init__("joint_state_estimator")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        self.joint_names = list(self.get_parameter("joint_names").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        self._positions = [0.0] * len(self.joint_names)

        self.create_subscription(JointState, "/joint_commands", self._on_joint_commands, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info(f"JointStateEstimator publishing /joint_states @ {rate} Hz")

    def _on_joint_commands(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)} if msg.name else {}
        for k, jn in enumerate(self.joint_names):
            if msg.name and jn in name_to_idx and len(msg.position) > name_to_idx[jn]:
                self._positions[k] = float(msg.position[name_to_idx[jn]])
            elif not msg.name and len(msg.position) == len(self.joint_names):
                self._positions = [float(x) for x in msg.position]
                break

    def _on_timer(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = self._positions
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
