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

def zyz_from_R(R):
    """
    Solve R = Rz(a) * Ry(b) * Rz(c)  -> returns (a,b,c).
    This matches your joint axes: joint4=Z, joint5=Y, joint6=Z.
    """
    r33 = float(R[2, 2])
    r33 = max(-1.0, min(1.0, r33))
    b = math.acos(r33)

    # singular when sin(b) ~ 0
    if abs(math.sin(b)) < 1e-9:
        a = math.atan2(float(R[1, 0]), float(R[0, 0]))
        c = 0.0
        return a, b, c

    a = math.atan2(float(R[1, 2]), float(R[0, 2]))       # atan2(r23, r13)
    c = math.atan2(float(R[2, 1]), -float(R[2, 0]))      # atan2(r32, -r31)
    return a, b, c


class MoveToolToPoseAnalyticSimple(Node):
    """
    Analytic IK for your specific kinematic chain:
      joint1: Z
      joint2: Y
      joint3: Y
      joint4: Z
      joint5: Y
      joint6: Z

    Assumption for "simple spherical wrist":
      L4 = L5 = L6 = 0  (all wrist rotations at the same point)
      no TCP/tool offset

    We solve:
      - position IK -> (q1,q2,q3) from target_xyz
      - orientation IK -> (q4,q5,q6) from target_rpy (optional)
    """

    def __init__(self):
        super().__init__("send_pose_trajectory")

        # Controller / joints
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names",
                               ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        # Target pose (base/world frame)
        self.declare_parameter("target_xyz", [0.0, 0.0, 0.30])
        self.declare_parameter("target_rpy", [0.0, 0.0, 0.0])

        # Robot geometry (must match URDF)
        self.declare_parameter("base_z", 0.04)
        self.declare_parameter("L1", 0.12)
        self.declare_parameter("L2", 0.12)
        self.declare_parameter("L3", 0.12)

        # Solution selection
        self.declare_parameter("elbow", "auto")  # "auto" | "up" | "down"
        self.declare_parameter("q_seed", [0, 0, 0, 0, 0, 0])

        # Optional: if True, we solve q4-6 to match target_rpy.
        # If False, we set q4=q5=q6=0 and only do position IK.
        self.declare_parameter("solve_orientation", True)

        # Trajectory send
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("wait_result", True)

        controller = self.get_parameter("controller_name").value
        self._action_name = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    # -----------------------------
    # FK (used only for orientation decoupling)
    # With L4=L5=L6=0, tool position depends only on q1-3.
    # R03 = Rz(q1) * Ry(q2+q3)
    # -----------------------------
    def R03(self, q1, q2, q3):
        return Rz(q1) @ Ry(q2 + q3)

    # -----------------------------
    # Analytic IK (position + optional orientation)
    # -----------------------------
    def ikine(self, target_xyz, target_rpy, q_seed):
        bz = float(self.get_parameter("base_z").value)
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)

        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])

        # 1) q1 from projection in XY
        q1 = math.atan2(y, x)

        # Reduce to planar 2R for q2,q3 in plane defined by q1:
        r = math.sqrt(x*x + y*y)
        z_plane = (z - bz) - L1

        # cosine law for elbow (q3)
        D = (r*r + z_plane*z_plane - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        if D < -1.0 - 1e-6 or D > 1.0 + 1e-6:
            return False, None, []

        D = max(-1.0, min(1.0, D))
        s = math.sqrt(max(0.0, 1.0 - D*D))

        # two elbow branches
        q3_down = math.atan2(+s, D)
        q3_up   = math.atan2(-s, D)

        def solve_q2(q3):
            # standard 2-link planar IK formula
            return math.atan2(r, z_plane) - math.atan2(L3*math.sin(q3), L2 + L3*math.cos(q3))

        q2_down = solve_q2(q3_down)
        q2_up   = solve_q2(q3_up)

        # orientation part
        solve_ori = bool(self.get_parameter("solve_orientation").value)
        Rd = rpy_to_R(float(target_rpy[0]), float(target_rpy[1]), float(target_rpy[2]))

        def make_solution(branch_name, q2, q3):
            if not solve_ori:
                q4, q5, q6 = 0.0, 0.0, 0.0
            else:
                R03 = self.R03(q1, q2, q3)
                R36 = R03.T @ Rd
                q4, q5, q6 = zyz_from_R(R36)

            q = wrap_vec([q1, q2, q3, q4, q5, q6])
            return branch_name, q

        sols = [
            make_solution("down", q2_down, q3_down),
            make_solution("up",   q2_up,   q3_up),
        ]

        elbow_mode = str(self.get_parameter("elbow").value).lower()
        q_seed = wrap_vec(q_seed)

        # choose requested elbow mode if specified
        if elbow_mode in ("up", "down"):
            for name, q in sols:
                if name == elbow_mode:
                    return True, q, sols

        # else auto: choose the solution closest to seed
        best_q = None
        best_cost = float("inf")
        for _, q in sols:
            dq = wrap_vec(q - q_seed)
            cost = float(np.linalg.norm(dq))
            if cost < best_cost:
                best_cost = cost
                best_q = q

        return True, best_q, sols

    # -----------------------------
    # Send a single trajectory point
    # -----------------------------
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
        q_seed = np.array(self.get_parameter("q_seed").value, dtype=float)

        ok, q_sol, sols = self.ikine(xyz, rpy, q_seed)
        if not ok or q_sol is None:
            self.get_logger().error("Analytic IK failed (unreachable target).")
            return 10

        self.get_logger().info("Analytic IK candidate solutions:")
        for name, q in sols:
            self.get_logger().info(f"  {name}: {q.tolist()}")
        self.get_logger().info(f"Selected q_sol={q_sol.tolist()}")

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
