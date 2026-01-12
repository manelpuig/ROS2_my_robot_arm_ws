# IK for `my_arm`

The inverse Kinematics finds the angles $q_i$ for speciffic desired pose $x_i$:
$$
q_i=IK(x_i)
$$
The IK method is proposed if your robot has:
- spherical wrist --> analytic method
- non-spherical wrist --> numeric method

## Spherical wrist (analytic IK)

A **spherical wrist** means the last three joint axes intersect at one point (the wrist center), and the last three joints are used mainly to set orientation.

**PUMA robot** has spherical wrist.

In many industrial arms, that occurs when the geometry satisfies something equivalent to:
- the wrist offsets are arranged so that the “wrist center” location depends only on $(q_1,q_2,q_3)$,
- and the tool offset ($d_6$) is a known constant along the tool $z$-axis.

Then the solution is decoupled:

1) **Position IK**: solve $(q_1,q_2,q_3)$ from the wrist center position ($p_{wc}$)
2) **Orientation IK**: solve $(q_4,q_5,q_6)$ from $R_{36} = R_{03}^T R_d$

Wrist center (conceptually):

$$
p_{wc} = p_d - d_6\;R_d\;\hat{z}
\qquad \hat{z}=\begin{bmatrix}0\\0\\1\end{bmatrix}
$$

Then solve the 3-DOF arm to reach $p_{wc}$, and finally:

$$
R_{36} = R_{03}^T R_d
$$

From $R_{36}$, the angles $(q_4,q_5,q_6)$ are extracted (depending on axis order).

**Important:** Your current toy-arm chain is not a clean spherical wrist in the strict industrial sense unless you enforce a geometry where the last three axes intersect and the offsets are arranged appropriately. That is why, for your current model, a robust “always works” approach is the **numeric IK** below.

---

# Without a Spherical Wrist: Numerical Inverse Kinematics (IK) 

This document complements the analytical inverse kinematics (IK) explanation previously developed for a PUMA-like robot with a spherical wrist. It focuses on **numerical inverse kinematics**, which is the standard approach when a clean geometric decoupling between position and orientation is not available.

The explanations are intentionally **didactic**, algorithmic, and suitable for implementation in a custom ROS 2 node (similar in spirit to the analytical PUMA IK node). The main focus is on **Jacobian-based methods**, in particular the **pseudoinverse formulation**, and their direct relation to the solvers used in MoveIt 2.

---

## 1. Orientation Error Representation

### 1.1 Position error

Positions belong to Euclidean space:
$$
p \in \mathbb{R}^3
$$
so the position error is naturally defined as:
$$
e_p = p_d - p(q)
$$


### 1.2 Relative rotation

The rotation that brings the current end-effector orientation $R(q)$ to the desired one $R_d$ is defined as:
$$
R_{err} = R(q)^T R_d
$$

Properties:
- $R_{err} = I$ if and only if the orientations match
- $R_{err}$ represents the orientation error independently of the world frame

---

### 1.3 Orientation error using angle–axis representation

For didactic clarity, the orientation error is expressed using the **angle–axis representation**, which is equivalent to the matrix logarithm but more intuitive.

Any rotation matrix $R_{err} \in SO(3)$ can be written as:
$$
R_{err} = \exp(\theta [\hat{u}]_\times)
$$
where:
- $\hat{u} \in \mathbb{R}^3$ is a unit rotation axis
- $\theta \in \mathbb{R}$ is the rotation angle (rad)
- $[\hat{u}]_\times$ is the skew-symmetric matrix associated with $\hat{u}$

The **orientation error vector** is then defined as:
$$
e_R = \theta \, \hat{u} \in \mathbb{R}^3
$$

Interpretation:
- Direction of $e_R$: axis of rotation
- Magnitude of $e_R$: rotation angle

This vector represents the **minimal rotation** required to align the current orientation with the desired one and is directly compatible with angular velocity.

---

## 2. Complete Pose Error Vector

Position and orientation errors are combined into a single 6D error vector:
$$
e = \begin{bmatrix} e_p \\ e_R \end{bmatrix} \in \mathbb{R}^6
$$

This vector represents the **twist error** of the end-effector.

---

## 3. Jacobian-Based Numerical Inverse Kinematics

### 3.1 Differential kinematics

The geometric Jacobian relates joint velocities to the end-effector twist:
$$
\dot{x} = \begin{bmatrix} v \\ \omega \end{bmatrix} = J(q)\, \dot{q}
$$

For small increments, this relation can be approximated as:
$$
\Delta x \approx J(q)\, \Delta q
$$

The objective of numerical IK is to compute $\Delta q$ such that the pose error $e$ is reduced.

---

## 4. Pseudoinverse Method (Resolved-Rate IK)

The **pseudoinverse method** is the most conceptually transparent numerical IK technique and is particularly well suited for teaching.

### 4.1 Core update equations

The joint increment is computed as:
$$
\Delta q = J(q)^+ \, e
$$

where $J^+$ is the Moore–Penrose pseudoinverse. For a non-square Jacobian:
$$
J^+ = J^T (J J^T)^{-1}
$$

The joint configuration is updated iteratively:
$$
q_{k+1} = q_k + \alpha \, \Delta q
$$

with $\alpha \in (0,1]$ a step size (gain).

---

### 4.2 Interpretation

- Computes the **minimum-norm joint motion** that reduces the pose error
- Solves a least-squares problem at each iteration
- Position and orientation are corrected **simultaneously**

---

## 5. Iterative Algorithm

### 5.1 Initialization (Seed)

The algorithm starts from an initial joint configuration:
$$
q_0 \quad \text{(seed)}
$$

The seed determines:
- Which local IK solution is found
- Continuity of motion
- Convergence behavior

---

### 5.2 Iterative loop

At iteration $k$:

1. **Forward kinematics**
$$
T(q_k) = \begin{bmatrix} R(q_k) & p(q_k) \\ 0 & 1 \end{bmatrix}
$$

2. **Compute pose error**
$$
e_p = p_d - p(q_k)
$$
$$
e_R = \theta \, \hat{u}
$$

3. **Compute Jacobian**
$$
J(q_k)
$$

4. **Compute joint increment**
$$
\Delta q_k = J(q_k)^+ \, e
$$

5. **Update joints**
$$
q_{k+1} = q_k + \alpha \, \Delta q_k
$$

---

### 5.3 Stopping criteria

The iteration stops when one of the following conditions is met:

1. **Pose convergence**
$$
\|e_p\| < \varepsilon_p \quad \text{and} \quad \|e_R\| < \varepsilon_R
$$

2. **Maximum number of iterations reached**
$$
k = k_{max}
$$

3. **Numerical stagnation**
$$
\|\Delta q_k\| < \varepsilon_q
$$

---

## 6. Relation to MoveIt 2 and KDL

MoveIt 2 primarily relies on **numerical Jacobian-based IK solvers**. A commonly used default solver is **KDL (Kinematics and Dynamics Library)**.

KDL:
- Uses the same differential kinematics principle described above
- Computes the Jacobian directly from the URDF-defined kinematic chain
- Solves the least-squares problem using **SVD-based pseudoinversion**
- Iterates from a seed until convergence

From a mathematical point of view, **KDL implements the same resolved-rate IK method**, differing mainly in:
- Numerical robustness
- Internal scaling and damping
- Better behavior near singularities

---

## 7. Comparison with Analytical IK (PUMA)

| PUMA (Spherical Wrist) | Numerical IK |
|----------------------|-------------|
| Position/orientation decoupled | Fully coupled |
| Closed-form equations | Iterative algorithm |
| Discrete solution branches | Seed-dependent solution |
| No Jacobian required | Jacobian required |
| Deterministic | Locally convergent |

---

## 8. Educational Takeaway

- Analytical IK explains **robot geometry**
- Numerical IK explains **robot motion and control**
- Jacobian-based IK provides the bridge between kinematics and control

Presenting both approaches side by side (analytical PUMA IK vs. numerical Jacobian-based IK) provides a coherent and complete framework for robotics education.

---

## 9. References

- B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer  
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, Pearson  
- R. Murray, Z. Li, S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press  
- Orocos KDL documentation  
- ROS 2 MoveIt documentation (Kinematics and IK solvers)
