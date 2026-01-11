#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


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


def zyz_from_R_two(R):
    """
    Decompose R = Rz(a) * Ry(b) * Rz(c), assuming joint4=Z, joint5=Y, joint6=Z.

    Returns TWO equivalent solutions (noflip, flip) when not singular.
    This gives you the classic "wrist flip / no-flip" branch.
    """
    r33 = float(R[2, 2])
    r33 = max(-1.0, min(1.0, r33))
    b = math.acos(r33)  # b in [0, pi]

    sb = math.sin(b)

    # singular when sin(b) ~ 0
    if abs(sb) < 1e-9:
        # Infinite solutions: we pick a simple one and duplicate it
        a = math.atan2(float(R[1, 0]), float(R[0, 0]))
        c = 0.0
        sol = (a, b, c)
        return sol, sol

    # One standard solution
    a1 = math.atan2(float(R[1, 2]), float(R[0, 2]))       # atan2(r23, r13)
    c1 = math.atan2(float(R[2, 1]), -float(R[2, 0]))      # atan2(r32, -r31)
    sol1 = (a1, b, c1)

    # The "flip" equivalent (valid for ZYZ; b can be negative if joints allow it)
    sol2 = (a1 + math.pi, -b, c1 + math.pi)

    return sol1, sol2


class MoveToolToPoseAnalyticSimple(Node):
    """
    Analytic IK for your academic simplified arm:
      joint1: Z
      joint2: Y
      joint3: Y
      joint4: Z
      joint5: Y
      joint6: Z

    Assumptions matching your xacro:
      - Spherical wrist: L4=L5=L6=0 (wrist center == tool point)
      - No TCP/tool offset
      - Shoulder offset d3 in +Y at joint2 origin
      - Base is shifted in world by base_z (world_to_base)

    Selection:
      - elbow: "up" | "down"  (no auto, no seed)
      - wrist: "noflip" | "flip"
    """

    def __init__(self):
        super().__init__("send_pose_trajectory")

        # Controller / joints
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names",
                               ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        # Target pose (world frame)
        self.declare_parameter("target_xyz", [0.25, 0.00, 0.25])
        self.declare_parameter("target_rpy", [0.0, 1.57, 0.0])

        # Geometry (must match xacro)
        self.declare_parameter("base_z", 0.02)
        self.declare_parameter("L1", 0.4)
        self.declare_parameter("L2", 0.4318)
        self.declare_parameter("L3", 0.43208)
        self.declare_parameter("d3", 0.1397)

        # Explicit solution selection
        self.declare_parameter("elbow", "down")     # "up" | "down"
        self.declare_parameter("wrist", "noflip")   # "noflip" | "flip"

        self.declare_parameter("solve_orientation", True)

        # Trajectory send
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("wait_result", True)

        controller = self.get_parameter("controller_name").value
        self._action_name = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    # Simplified orientation decoupling
    # With your model: R03 = Rz(q1) * Ry(q2+q3)
    def R03(self, q1, q2, q3):
        return Rz(q1) @ Ry(q2 + q3)

    def ikine(self, target_xyz, target_rpy):
        bz = float(self.get_parameter("base_z").value)
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        d3 = float(self.get_parameter("d3").value)

        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])

        # --- Solve q1 with shoulder offset d3 ---
        # In XY plane, shoulder is at radius d3 from z-axis. Let r be target radius.
        r = math.sqrt(x*x + y*y)
        if r < abs(d3) - 1e-9:
            return False, None, []

        # Choose the common "PUMA-like" branch for q1 (one of possible shoulder branches)
        phi = math.atan2(y, x)
        gamma = math.atan2(d3, math.sqrt(max(0.0, r*r - d3*d3)))
        q1 = phi - gamma

        # Planar distance from shoulder to target in the arm plane
        x_plane = math.sqrt(max(0.0, r*r - d3*d3))
        z_plane = z - L1

        # --- Elbow (q3) via cosine law ---
        D = (x_plane*x_plane + z_plane*z_plane - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        if D < -1.0 - 1e-6 or D > 1.0 + 1e-6:
            return False, None, []

        D = max(-1.0, min(1.0, D))
        s = math.sqrt(max(0.0, 1.0 - D*D))

        q3_down = math.atan2(+s, D)
        q3_up   = math.atan2(-s, D)

        def solve_q2(q3):
            # Standard 2-link planar IK in (x_plane, z_plane)
            return math.atan2(z_plane, x_plane) - math.atan2(L3*math.sin(q3), L2 + L3*math.cos(q3))

        q2_down = solve_q2(q3_down)
        q2_up   = solve_q2(q3_up)

        solve_ori = bool(self.get_parameter("solve_orientation").value)
        Rd = rpy_to_R(float(target_rpy[0]), float(target_rpy[1]), float(target_rpy[2]))

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

        candidates = [
            make_solution("down", q2_down, q3_down, "noflip"),
            make_solution("down", q2_down, q3_down, "flip"),
            make_solution("up",   q2_up,   q3_up,   "noflip"),
            make_solution("up",   q2_up,   q3_up,   "flip"),
        ]

        elbow = str(self.get_parameter("elbow").value).lower()
        wrist = str(self.get_parameter("wrist").value).lower()

        if elbow not in ("up", "down"):
            self.get_logger().error("Parameter 'elbow' must be 'up' or 'down'.")
            return False, None, candidates

        if wrist not in ("noflip", "flip"):
            self.get_logger().error("Parameter 'wrist' must be 'noflip' or 'flip'.")
            return False, None, candidates

        # Select exactly the requested branch (no auto)
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

        self.get_logger().info(f"Sending q={pt.positions}")
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

    def run_once(self):
        xyz = np.array(self.get_parameter("target_xyz").value, dtype=float)
        rpy = np.array(self.get_parameter("target_rpy").value, dtype=float)

        ok, q_sol, cands = self.ikine(xyz, rpy)
        if not ok or q_sol is None:
            self.get_logger().error("Analytic IK failed (unreachable target or invalid params).")
            self.get_logger().info("Candidate solutions (if any):")
            for e, w, q in cands:
                self.get_logger().info(f"  elbow={e}, wrist={w}: {q.tolist()}")
            return 10

        self.get_logger().info("Analytic IK candidate solutions:")
        for e, w, q in cands:
            self.get_logger().info(f"  elbow={e}, wrist={w}: {q.tolist()}")

        self.get_logger().info(
            f"Selected: elbow={self.get_parameter('elbow').value}, "
            f"wrist={self.get_parameter('wrist').value}, "
            f"q={q_sol.tolist()}"
        )

        return self.send_trajectory(q_sol)


def main():
    rclpy.init()
    node = MoveToolToPoseAnalyticSimple()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)
