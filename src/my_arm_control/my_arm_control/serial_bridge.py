#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("write_rate_hz", 50.0)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("mode", "angles_rad")  # keep simple
        self.declare_parameter("line_prefix", "J:")

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.line_prefix = self.get_parameter("line_prefix").get_parameter_value().string_value

        self._last_positions = [0.0] * len(self.joint_names)

        self._ser = None
        self._connect_serial()

        self.create_subscription(JointState, "/joint_commands", self._on_joint_commands, 10)

        self.get_logger().info(
            f"SerialBridge ready. port={self.port} baud={self.baud} joints={self.joint_names}"
        )

    def _connect_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f"Connected to serial: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self._ser = None

    def _on_joint_commands(self, msg: JointState):
        # Map incoming msg positions to expected joint order
        name_to_idx = {n: i for i, n in enumerate(msg.name)} if msg.name else {}
        out = []
        for jn in self.joint_names:
            if msg.name and jn in name_to_idx and len(msg.position) > name_to_idx[jn]:
                out.append(float(msg.position[name_to_idx[jn]]))
            else:
                # fallback: use last known value for missing joints
                out.append(self._last_positions[self.joint_names.index(jn)])

        self._last_positions = out

        if self._ser is None:
            self._connect_serial()
            return

        # ASCII protocol: "J:a1,a2,a3,a4,a5,a6\n"
        line = self.line_prefix + ",".join(f"{v:.6f}" for v in out) + "\n"
        try:
            self._ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        if node._ser:
            try:
                node._ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
