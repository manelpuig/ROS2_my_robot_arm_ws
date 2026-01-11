#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# -----------------------------
# 4x4 homogeneous transforms
# -----------------------------
def transl(x, y, z):
    T = np.eye(4, dtype=float)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def trotx(a):
    ca, sa = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [1,  0,   0],
        [0, ca, -sa],
        [0, sa,  ca]
    ], float)
    return T

def troty(a):
    ca, sa = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [ ca, 0, sa],
        [  0, 1,  0],
        [-sa, 0, ca]
    ], float)
    return T

def trotz(a):
    ca, sa = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [ca, -sa, 0],
        [sa,  ca, 0],
        [ 0,   0, 1]
    ], float)
    return T

def rpy_to_T(roll, pitch, yaw, x, y, z):
    # Z-Y-X convention for RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return transl(x, y, z) @ trotz(yaw) @ troty(pitch) @ trotx(roll)

def rotvec_from_R(R):
    """
    Return axis-angle vector (rotation vector) w:
      - direction(w) is axis
      - ||w|| is angle (rad)
    """
    tr = float(np.trace(R))
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    theta = math.acos(c)

    if theta < 1e-9:
        return np.zeros(3, float)

    wx = (R[2, 1] - R[1, 2]) / (2.0 * math.sin(theta))
    wy = (R[0, 2] - R[2, 0]) / (2.0 * math.sin(theta))
    wz = (R[1, 0] - R[0, 1]) / (2.0 * math.sin(theta))
    axis = np.array([wx, wy, wz], float)

    return axis * theta

class MoveToolToPoseSimple(Node):
    def __init__(self):
        super().__init__("send_pose_trajectory")

        # Controller / joints
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("joint_names", ["joint1","joint2","joint3","joint4","joint5","joint6"])

        # Target pose in base/world frame
        self.declare_parameter("target_xyz", [0.0, 0.0, 0.45])
        self.declare_parameter("target_rpy", [0.0, 0.0, 0.0])

        # Robot geometry (must match your URDF)
        self.declare_parameter("base_z", 0.04)
        self.declare_parameter("L1", 0.12)
        self.declare_parameter("L2", 0.12)
        self.declare_parameter("L3", 0.12)
        self.declare_parameter("L4", 0.10)
        self.declare_parameter("L5", 0.08)
        self.declare_parameter("L6", 0.06)

        # IK parameters
        self.declare_parameter("q_seed", [0, 0, 0, 0, 0, 0])
        self.declare_parameter("max_iters", 120)
        self.declare_parameter("tol_pos", 1e-3)
        self.declare_parameter("tol_rot", 1e-2)
        self.declare_parameter("eps_jac", 1e-5)
        self.declare_parameter("alpha", 0.6)      # step size
        self.declare_parameter("damping", 0.05)   # DLS damping

        # Trajectory send
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("wait_result", True)

        controller = self.get_parameter("controller_name").value
        self._action_name = f"/{controller}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    # ---------------- FK ----------------
    def fkine(self, q):
        bz = float(self.get_parameter("base_z").value)
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        L4 = float(self.get_parameter("L4").value)
        L5 = float(self.get_parameter("L5").value)
        L6 = float(self.get_parameter("L6").value)

        # Must match URDF joint axes and offsets:
        # joint1: Rz, then +L1
        # joint2: Ry, then +L2
        # joint3: Ry, then +L3
        # joint4: Rz, then +L4
        # joint5: Ry, then +L5
        # joint6: Rz, then +L6 (tool offset)
        T = np.eye(4, dtype=float)
        T = T @ transl(0, 0, bz)

        T = T @ trotz(q[0]) @ transl(0, 0, L1)
        T = T @ troty(q[1]) @ transl(0, 0, L2)
        T = T @ troty(q[2]) @ transl(0, 0, L3)
        T = T @ trotz(q[3]) @ transl(0, 0, L4)
        T = T @ troty(q[4]) @ transl(0, 0, L5)
        T = T @ trotz(q[5]) @ transl(0, 0, L6)

        return T

    # ---------------- IK ----------------
    def ikine(self, T_des, q_seed):
        """
        Numerical IK using:
        - 6D pose error e = [position_error; orientation_error]
        - numerical Jacobian J (finite differences)
        - Damped Least Squares (DLS) to compute dq step

        Returns:
        ok (bool): True if converged
        q  (np.ndarray): solution (or best effort)
        iters (int): iterations used
        """
        max_iters = int(self.get_parameter("max_iters").value)
        tol_pos   = float(self.get_parameter("tol_pos").value)
        tol_rot   = float(self.get_parameter("tol_rot").value)
        eps       = float(self.get_parameter("eps_jac").value)
        alpha     = float(self.get_parameter("alpha").value)
        lam       = float(self.get_parameter("damping").value)

        q = q_seed.astype(float).copy()

        for k in range(max_iters):
            # Current pose from FK
            Tcur = self.fkine(q)
            p = Tcur[:3, 3]
            R = Tcur[:3, :3]

            # Desired pose
            pd = T_des[:3, 3]
            Rd = T_des[:3, :3]

            # 1) Compute 6D error vector e = [e_p; e_w]
            e_p = (pd - p)  # position error in base frame

            # orientation error:
            # R_err = R^T Rd is "rotation still needed" from current to desired
            R_err = R.T @ Rd
            e_w_local = rotvec_from_R(R_err)  # axis-angle vector (local)
            e_w = R @ e_w_local               # express in base frame

            e = np.concatenate((e_p, e_w), axis=0)

            # Convergence check
            if np.linalg.norm(e_p) < tol_pos and np.linalg.norm(e_w) < tol_rot:
                return True, q, k

            # 2) Numerical Jacobian J (finite differences)
            # J[:,i] ≈ (x(q+eps*ei) - x(q))/eps   for pose x = [p; orientation]
            J = np.zeros((6, 6), float)
            for i in range(6):
                dq = np.zeros(6, float)
                dq[i] = eps
                T1 = self.fkine(q + dq)

                p1 = T1[:3, 3]
                R1 = T1[:3, :3]

                dp = (p1 - p) / eps

                # small orientation change from R to R1:
                # Rerr1 = R^T R1  -> "incremental" rotation due to dq[i]
                Rerr1 = R.T @ R1
                dw_local = rotvec_from_R(Rerr1) / eps
                dw = R @ dw_local

                J[:, i] = np.concatenate((dp, dw), axis=0)

            # 3) Damped Least Squares step to compute dq
            # dq = J^T (J J^T + λ^2 I)^-1 e
            JJt = J @ J.T
            dq_step = J.T @ np.linalg.solve(JJt + (lam ** 2) * np.eye(6,6), e)

            # Update
            q = q + alpha * dq_step

        return False, q, max_iters


    # ---------------- Send trajectory ----------------
    def send_trajectory(self, q_target):
        joint_names = list(self.get_parameter("joint_names").value)
        duration = float(self.get_parameter("duration_sec").value)
        wait_result = bool(self.get_parameter("wait_result").value)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_target.tolist()]
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
        xyz = np.array(self.get_parameter("target_xyz").value, float)
        rpy = np.array(self.get_parameter("target_rpy").value, float)
        T_des = rpy_to_T(rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2])

        q_seed = np.array(self.get_parameter("q_seed").value, float)

        ok, q_sol, iters = self.ikine(T_des, q_seed)
        self.get_logger().info(f"IK ok={ok} iters={iters} q_sol={q_sol.tolist()}")
        if not ok:
            self.get_logger().warn("IK did not converge within tolerances; sending best-effort anyway.")

        return self.send_trajectory(q_sol)


def main():
    rclpy.init()
    node = MoveToolToPoseSimple()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)
