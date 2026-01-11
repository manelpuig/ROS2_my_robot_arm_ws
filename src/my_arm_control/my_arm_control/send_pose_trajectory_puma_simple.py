#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import tf2_ros
from tf_transformations import euler_from_quaternion


def Rz(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ca, -sa, 0.0],
                     [sa,  ca, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

def Ry(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ ca, 0.0, sa],
                     [0.0, 1.0, 0.0],
                     [-sa, 0.0, ca]], dtype=float)

def Rx(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0,  ca, -sa],
                     [0.0,  sa,  ca]], dtype=float)

def rpy_to_R(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def wrap_vec(q):
    return np.array([wrap_to_pi(float(x)) for x in q], dtype=float)

def rad2deg_vec(q):
    return [float(x * 180.0 / math.pi) for x in q]

def deg2rad_vec(q):
    return [float(x * math.pi / 180.0) for x in q]

def zyz_from_R(R):
    r33 = float(R[2, 2])
    r33 = max(-1.0, min(1.0, r33))
    b = math.acos(r33)

    if abs(math.sin(b)) < 1e-9:
        a = math.atan2(float(R[1, 0]), float(R[0, 0]))
        c = 0.0
        return a, b, c

    a = math.atan2(float(R[1, 2]), float(R[0, 2]))
    c = math.atan2(float(R[2, 1]), -float(R[2, 0]))
    return a, b, c


def wrap_to_limits_rad(q, q_min, q_max):
    """
    Old behavior: try q +/- 2*pi*k with k in [-2..2].
    If limits are wide or missing, this may accept 'inflated' angles.
    """
    q_adj = np.array(q, dtype=float)
    for i in range(len(q_adj)):
        qi = wrap_to_pi(q_adj[i])
        if q_min[i] <= qi <= q_max[i]:
            q_adj[i] = qi
            continue

        ok = False
        for k in range(-2, 3):
            cand = qi + 2.0 * math.pi * k
            if q_min[i] <= cand <= q_max[i]:
                q_adj[i] = cand
                ok = True
                break

        if not ok:
            return False, np.array(q, dtype=float)

    return True, q_adj


class SendPoseTrajectoryPuma(Node):
    def __init__(self):
        super().__init__("send_pose_trajectory")

        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names",
                               ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        self.declare_parameter("target_xyz", [0.35, 0.0, 0.55])
        self.declare_parameter("target_rpy_deg", [0.0, 0.0, 0.0])

        self.declare_parameter("L1", 0.4)
        self.declare_parameter("L2", 0.4318)
        self.declare_parameter("L3", 0.43208)
        self.declare_parameter("d3", 0.1397)

        self.declare_parameter("elbow", "up")
        self.declare_parameter("wrist", "noflip")
        self.declare_parameter("solve_orientation", False)

        # Limits are optional here (wide defaults)
        self.declare_parameter("joint_lower_deg", [-180, -180, -180, -180, -180, -180])
        self.declare_parameter("joint_upper_deg", [ 180,  180,  180,  180,  180,  180])

        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("wait_result", True)

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("tf_timeout_sec", 2.0)

        controller = self.get_parameter("controller_name").value
        self._action_name = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

        self._tfbuf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tfbuf, self)

    def R03(self, q1, q2, q3):
        return Rz(q1) @ Ry(q2 + q3)

    def ikine(self, target_xyz, target_rpy_deg):
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        d3 = float(self.get_parameter("d3").value)

        q_min = np.array(deg2rad_vec(self.get_parameter("joint_lower_deg").value), dtype=float)
        q_max = np.array(deg2rad_vec(self.get_parameter("joint_upper_deg").value), dtype=float)

        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])

        # shoulder-left (d3 in +Y): q1 = phi - gamma
        r = math.sqrt(x*x + y*y)
        if r < abs(d3) + 1e-9:
            return False, None, "Target too close to base axis for given d3."
        r_xy = math.sqrt(max(0.0, r*r - d3*d3))
        phi = math.atan2(y, x)
        gamma = math.atan2(d3, r_xy)
        q1 = phi - gamma

        x_p = r_xy
        z_p = z - L1

        R = math.sqrt(x_p*x_p + z_p*z_p)
        if R < 1e-9:
            return False, None, "Degenerate target (R ~ 0)."

        gamma2 = math.atan2(x_p, z_p)
        C = (x_p*x_p + z_p*z_p + L3*L3 - L2*L2) / (2.0 * L3 * R)
        if C < -1.0 - 1e-6 or C > 1.0 + 1e-6:
            return False, None, "Unreachable target."

        C = max(-1.0, min(1.0, C))
        delta = math.acos(C)

        phi_a = gamma2 + delta
        phi_b = gamma2 - delta

        def solve_q2q3_from_phi(phi_total):
            cx = (x_p - L3 * math.sin(phi_total)) / L2
            sz = -(z_p - L3 * math.cos(phi_total)) / L2
            q2 = math.atan2(sz, cx)
            q3 = wrap_to_pi(phi_total - q2)
            return wrap_to_pi(q2), q3

        q2_a, q3_a = solve_q2q3_from_phi(phi_a)
        q2_b, q3_b = solve_q2q3_from_phi(phi_b)

        elbow_mode = str(self.get_parameter("elbow").value).lower()
        if elbow_mode == "up":
            q2, q3 = q2_a, q3_a
        elif elbow_mode == "down":
            q2, q3 = q2_b, q3_b
        else:
            return False, None, "Invalid elbow (use up/down)."

        solve_ori = bool(self.get_parameter("solve_orientation").value)
        if not solve_ori:
            q4 = q5 = q6 = 0.0
        else:
            roll, pitch, yaw = deg2rad_vec(list(target_rpy_deg))
            Rd = rpy_to_R(roll, pitch, yaw)
            R03 = self.R03(q1, q2, q3)
            R36 = R03.T @ Rd
            q4, q5, q6 = zyz_from_R(R36)

            wrist_mode = str(self.get_parameter("wrist").value).lower()
            if wrist_mode == "noflip":
                pass
            elif wrist_mode == "flip":
                q4 = q4 + math.pi
                q5 = -q5
                q6 = q6 + math.pi
            else:
                return False, None, "Invalid wrist (use noflip/flip)."

        q = wrap_vec([q1, q2, q3, q4, q5, q6])

        ok, q_adj = wrap_to_limits_rad(q, q_min, q_max)
        if not ok:
            return False, None, f"IK violates limits. q_deg={rad2deg_vec(q)}"

        return True, q_adj, None

    def send_trajectory(self, q_target):
        joint_names = list(self.get_parameter("joint_names").value)
        duration = float(self.get_parameter("duration_sec").value)
        wait_result = bool(self.get_parameter("wait_result").value)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_target.tolist()]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [pt]

        self.get_logger().info(f"Waiting for action server: {self._action_name}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available.")
            return 2

        self.get_logger().info(f"Selected q_deg={rad2deg_vec(q_target)}")
        self.get_logger().info(f"Sending q (rad)={pt.positions}")

        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected.")
            return 3

        if not wait_result:
            return 0

        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        res = rfut.result()
        if res is None:
            self.get_logger().error("No result received.")
            return 4

        self.get_logger().info(f"Result status={res.status}, error_code={res.result.error_code}")
        return 0 if res.result.error_code == 0 else 5

    def tf_verify(self):
        base_link = str(self.get_parameter("base_link").value)
        tip_link = str(self.get_parameter("tip_link").value)
        timeout = float(self.get_parameter("tf_timeout_sec").value)

        try:
            t = self._tfbuf.lookup_transform(
                base_link, tip_link, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except Exception as e:
            self.get_logger().warn(f"TF verify failed: {e}")
            return

        tr = t.transform.translation
        q = t.transform.rotation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        rpy_deg = [x * 180.0 / math.pi for x in rpy]

        self.get_logger().info(
            f"TF verify pose {base_link} -> {tip_link}: "
            f"xyz=[{tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f}] m, "
            f"rpy=[{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}] deg"
        )

    def run_once(self):
        xyz = np.array(self.get_parameter("target_xyz").value, dtype=float)
        rpy_deg = np.array(self.get_parameter("target_rpy_deg").value, dtype=float)

        ok, q_sol, err = self.ikine(xyz, rpy_deg)
        if not ok or q_sol is None:
            self.get_logger().error(f"Analytic IK failed: {err}")
            return 10

        rc = self.send_trajectory(q_sol)
        if rc == 0:
            self.tf_verify()
        return rc


def main():
    rclpy.init()
    node = SendPoseTrajectoryPuma()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)
