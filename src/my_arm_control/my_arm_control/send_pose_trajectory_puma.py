#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


# -----------------------------
# Basic rotations
# -----------------------------
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
    # Z-Y-X convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def wrap_vec(q):
    return np.array([wrap_to_pi(float(x)) for x in q], dtype=float)

def deg2rad(v):
    return np.deg2rad(np.array(v, dtype=float))

def rad2deg(v):
    return np.rad2deg(np.array(v, dtype=float))


def zyz_from_R_two(R):
    """
    Decompose R = Rz(a) * Ry(b) * Rz(c), matching joint4=Z, joint5=Y, joint6=Z.
    Returns two equivalent solutions: (noflip, flip).
    """
    r33 = float(R[2, 2])
    r33 = max(-1.0, min(1.0, r33))
    b = math.acos(r33)  # b in [0, pi]

    sb = math.sin(b)
    if abs(sb) < 1e-9:
        a = math.atan2(float(R[1, 0]), float(R[0, 0]))
        c = 0.0
        sol = (a, b, c)
        return sol, sol

    a1 = math.atan2(float(R[1, 2]), float(R[0, 2]))       # atan2(r23, r13)
    c1 = math.atan2(float(R[2, 1]), -float(R[2, 0]))      # atan2(r32, -r31)
    sol1 = (a1, b, c1)

    # "flip" equivalent (ZYZ)
    sol2 = (a1 + math.pi, -b, c1 + math.pi)

    return sol1, sol2


class MoveToolToPoseAnalyticPumaSimple(Node):
    """
    Analytic IK for simplified PUMA-like academic arm (spherical wrist):
      joint1: Z
      joint2: Y
      joint3: Y
      joint4: Z
      joint5: Y
      joint6: Z

    Requested modifications:
      - elbow up/down swapped (to match your URDF visual convention)
      - target_xyz is expressed in base_link frame
      - target_rpy is provided in degrees (target_rpy_deg)
      - logs output joint solutions in degrees
      - after execution, reads TF (from robot_state_publisher fed by joint_state_broadcaster)
        and logs the final pose base_link -> tip_link

    Notes:
      - You must have joint_state_broadcaster running AND robot_state_publisher publishing TF.
      - tip_link default is "link6" (last link in your xacro).
    """

    def __init__(self):
        super().__init__("send_pose_trajectory")

        # Controller / joints
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names",
                               ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        # Target pose in BASE_LINK frame
        self.declare_parameter("target_xyz", [0.25, 0.00, 0.45])  # meters, base_link
        self.declare_parameter("target_rpy_deg", [0.0, 90.0, 0.0])  # degrees, base_link

        # Geometry (match xacro)
        self.declare_parameter("L1", 0.4)
        self.declare_parameter("L2", 0.4318)
        self.declare_parameter("L3", 0.43208)
        self.declare_parameter("d3", 0.1397)

        # Explicit branch selection
        self.declare_parameter("elbow", "down")     # "up" | "down" (already swapped)
        self.declare_parameter("wrist", "noflip")   # "noflip" | "flip"

        self.declare_parameter("solve_orientation", True)

        # Trajectory send
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("wait_result", True)

        # TF verification
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("tf_timeout_sec", 2.0)

        controller = self.get_parameter("controller_name").value
        self._action_name = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

        # TF listener for verification (requires robot_state_publisher publishing TF)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # With simplified model: R03 = Rz(q1) * Ry(q2+q3)
    def R03(self, q1, q2, q3):
        return Rz(q1) @ Ry(q2 + q3)

    def ikine(self, target_xyz, target_rpy_deg):
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        d3 = float(self.get_parameter("d3").value)

        # Target is in base_link frame (no world/base conversion)
        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])

        # --- Solve q1 with shoulder offset d3 ---
        r = math.sqrt(x*x + y*y)
        if r < abs(d3) - 1e-9:
            return False, None, []

        phi = math.atan2(y, x)
        gamma = math.atan2(d3, math.sqrt(max(0.0, r*r - d3*d3)))
        q1 = phi - gamma

        # Planar coordinates from shoulder to target
        x_plane = math.sqrt(max(0.0, r*r - d3*d3))
        z_plane = z - L1

        # --- Elbow (q3) via cosine law ---
        D = (x_plane*x_plane + z_plane*z_plane - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        if D < -1.0 - 1e-6 or D > 1.0 + 1e-6:
            return False, None, []

        D = max(-1.0, min(1.0, D))
        s = math.sqrt(max(0.0, 1.0 - D*D))

        q3_down_math = math.atan2(+s, D)
        q3_up_math   = math.atan2(-s, D)

        def solve_q2(q3):
            return math.atan2(z_plane, x_plane) - math.atan2(L3*math.sin(q3), L2 + L3*math.cos(q3))

        q2_down_math = solve_q2(q3_down_math)
        q2_up_math   = solve_q2(q3_up_math)

        solve_ori = bool(self.get_parameter("solve_orientation").value)
        rpy = deg2rad(target_rpy_deg)
        Rd = rpy_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        def make_solution(elbow_name, q2, q3, wrist_name):
            if not solve_ori:
                q4, q5, q6 = 0.0, 0.0, 0.0
            else:
                R03 = self.R03(q1, q2, q3)
                R36 = R03.T @ Rd
                (a1, b1, c1), (a2, b2, c2) = zyz_from_R_two(R36)
                if wrist_name == "noflip":
                    q4, q5, q6 = a1, b1, c1
                else:
                    q4, q5, q6 = a2, b2, c2

            q = wrap_vec([q1, q2, q3, q4, q5, q6])
            return (elbow_name, wrist_name, q)

        # --- IMPORTANT: swap elbow labels to match your URDF visual convention ---
        # Mathematical "down" is labeled as "up", and mathematical "up" as "down".
        candidates = [
            make_solution("up",   q2_down_math, q3_down_math, "noflip"),
            make_solution("up",   q2_down_math, q3_down_math, "flip"),
            make_solution("down", q2_up_math,   q3_up_math,   "noflip"),
            make_solution("down", q2_up_math,   q3_up_math,   "flip"),
        ]

        elbow = str(self.get_parameter("elbow").value).lower()
        wrist = str(self.get_parameter("wrist").value).lower()

        if elbow not in ("up", "down"):
            self.get_logger().error("Parameter 'elbow' must be 'up' or 'down'.")
            return False, None, candidates
        if wrist not in ("noflip", "flip"):
            self.get_logger().error("Parameter 'wrist' must be 'noflip' or 'flip'.")
            return False, None, candidates

        for e, w, q in candidates:
            if e == elbow and w == wrist:
                return True, q, candidates

        return False, None, candidates

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

        self.get_logger().info(f"Sending q (rad)={pt.positions}")
        self.get_logger().info(f"Sending q (deg)={rad2deg(np.array(pt.positions)).tolist()}")

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

    def log_final_pose_from_tf(self):
        base_link = str(self.get_parameter("base_link").value)
        tip_link = str(self.get_parameter("tip_link").value)
        tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        # Attempt to read TF base_link -> tip_link
        try:
            tf = self._tf_buffer.lookup_transform(
                base_link,
                tip_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not lookup TF {base_link} -> {tip_link}. "
                f"Make sure joint_state_broadcaster + robot_state_publisher are running. "
                f"Error: {e}"
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        # Convert quaternion to RPY (rad) then to degrees
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        rpy_deg = rad2deg(np.array([roll, pitch, yaw]))

        self.get_logger().info(
            f"TF verify pose {base_link} -> {tip_link}: "
            f"xyz=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}] m, "
            f"rpy=[{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}] deg"
        )

    def run_once(self):
        xyz = np.array(self.get_parameter("target_xyz").value, dtype=float)
        rpy_deg = np.array(self.get_parameter("target_rpy_deg").value, dtype=float)

        ok, q_sol, cands = self.ikine(xyz, rpy_deg)
        if not ok or q_sol is None:
            self.get_logger().error("Analytic IK failed (unreachable target or invalid params).")
            self.get_logger().info("Candidate solutions (deg):")
            for e, w, q in cands:
                self.get_logger().info(f"  elbow={e}, wrist={w}: {rad2deg(q).tolist()}")
            return 10

        self.get_logger().info("Analytic IK candidate solutions (deg):")
        for e, w, q in cands:
            self.get_logger().info(f"  elbow={e}, wrist={w}: {rad2deg(q).tolist()}")

        self.get_logger().info(
            f"Selected: elbow={self.get_parameter('elbow').value}, "
            f"wrist={self.get_parameter('wrist').value}, "
            f"q_deg={rad2deg(q_sol).tolist()}"
        )

        rc = self.send_trajectory(q_sol)

        # Verification: log final link pose from TF after execution
        self.log_final_pose_from_tf()

        return rc


def main():
    rclpy.init()
    node = MoveToolToPoseAnalyticPumaSimple()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)
