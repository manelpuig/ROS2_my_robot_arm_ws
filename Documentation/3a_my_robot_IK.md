# IK for `my_arm`: spherical wrist (analytic) vs non-spherical wrist (numeric)

This note matches the kinematic conventions used in the ROS 2 node you are running (`move_tool_to_pose.py`):
- Joint axes (as in your URDF / FK in code):

    $(q_1:\,R_z), (q_2:\,R_y), (q_3:\,R_y), (q_4:\,R_z), (q_5:\,R_y), (q_6:\,R_z)$
- Link offsets are pure translations along local \(+z\): \(L_1,\dots,L_6\) (and `base_z`).
- FK in code (same order):

$$
T = T_z(b_z)\;R_z(q_1)T_z(L_1)\;R_y(q_2)T_z(L_2)\;R_y(q_3)T_z(L_3)\;R_z(q_4)T_z(L_4)\;R_y(q_5)T_z(L_5)\;R_z(q_6)T_z(L_6)
$$

where \(T_z(d)=\text{transl}(0,0,d)\).

---

## 1) Poses and notation

We use standard homogeneous transforms:

$$
T = \begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
\quad\text{with}\quad
R\in SO(3),\; p\in\mathbb{R}^3
$$

Desired pose:

$$
T_d = \begin{bmatrix}
R_d & p_d \\
0 & 1
\end{bmatrix}
$$

Current pose (from FK at current joint vector \(q\)):

$$
T(q) = \begin{bmatrix}
R(q) & p(q) \\
0 & 1
\end{bmatrix}
$$

---

## 2) When you *do* have a spherical wrist (analytic IK idea)

A **spherical wrist** means the last three joint axes intersect at one point (the wrist center), and the last three joints are used mainly to set orientation.

In many industrial arms, that occurs when the geometry satisfies something equivalent to:
- the wrist offsets are arranged so that the “wrist center” location depends only on \(q_1,q_2,q_3\),
- and the tool offset is a known constant along the tool \(z\)-axis.

Then the standard split is:

1) **Position IK**: solve $(q_1,q_2,q_3)$ from the wrist center position  
2) **Orientation IK**: solve $(q_4,q_5,q_6)$ from $R_{36} = R_{03}^T R_d$

Wrist center (conceptually):

$$
p_{wc} = p_d - d_6\;R_d\;\hat{z}
\qquad \hat{z}=\begin{bmatrix}0\\0\\1\end{bmatrix}
$$

Then solve the 3-DOF arm to reach \(p_{wc}\), and finally:

$$
R_{36} = R_{03}^T R_d
$$

From $R_{36}$, the angles $(q_4,q_5,q_6)$ are extracted (depending on axis order).

**Important:** Your current toy-arm chain is not a clean spherical wrist in the strict industrial sense unless you enforce a geometry where the last three axes intersect and the offsets are arranged appropriately. That is why, for your current model, a robust “always works” approach is the **numeric IK** below.

---

## 3) When you *do not* have a spherical wrist: what the node does (numeric IK)

When the wrist is not spherical (or the link offsets are arbitrary), the “position/orientation split” breaks:
- orientation and position are coupled across all joints,
- there is no clean wrist-center trick,
- closed-form (analytic) IK is either unavailable or becomes very case-specific.

So the node uses a **general numerical IK** that works for any serial chain (as long as FK is correct).

### 3.1 Pose error used in the node

The node builds a 6D error vector:

$$
e = \begin{bmatrix} e_p \\ e_\omega \end{bmatrix}\in\mathbb{R}^6
$$

**Position error:**

$$
e_p = p_d - p
$$

**Orientation error:**

1) Compute the rotation still needed from current to desired in the *current* frame:

$$
R_{err} = R^T R_d
$$

2) Convert \(R_{err}\) to a rotation vector (axis-angle vector)

$$
\omega_{local} = \text{rotvec}(R_{err})
$$

This is exactly what your `rotvec_from_R()` computes:
- direction is rotation axis
- norm is rotation angle (rad)

3) Express it back in the base frame (this matches the code line `e_w = R @ e_w_local`):

$$
e_\omega = R\;\omega_{local}
$$

So the final error is:

$$
e = \begin{bmatrix} p_d - p \\ R\;\text{rotvec}(R^T R_d) \end{bmatrix}
$$

**Interpretation (simple):**
- \(e_p\) says “how far the tool tip is from the target position” (meters)
- \(e_\omega\) says “what small rotation (as an angle-vector) is needed to align the tool orientation with the target” (radians)

---

### 3.2 Numerical Jacobian used in the node

We want a Jacobian \(J\in\mathbb{R}^{6\times 6}\) such that for small changes:

$$
\Delta x \approx J\;\Delta q
\quad\text{where}\quad
x=\begin{bmatrix}p\\\omega\end{bmatrix}
$$

Because we do not derive \(J\) analytically, the code uses **finite differences**:

For each joint \(i\):
- perturb the joint by \(\varepsilon\): \(q' = q + \varepsilon e_i\)
- compute FK again: \(T(q')\)
- approximate derivatives:

$$
\frac{\partial p}{\partial q_i}\approx \frac{p(q+\varepsilon e_i)-p(q)}{\varepsilon}
$$

For orientation, the code computes the incremental rotation from \(R\) to \(R'\):

$$
R_{\Delta} = R^T R'
$$

Then converts it to a rotation vector and divides by \(\varepsilon\):

$$
\frac{\partial \omega}{\partial q_i}\approx \frac{\text{rotvec}(R^T R')}{\varepsilon}
$$

Finally it maps to base frame the same way:

$$
\left(\frac{\partial \omega}{\partial q_i}\right)_{base} \approx R\;\frac{\text{rotvec}(R^T R')}{\varepsilon}
$$

That produces the Jacobian column \(J[:,i]\).

**Why numerical Jacobian is used (and why it is reasonable here):**
- It avoids mistakes when FK / axes change while you iterate on the URDF
- It is compact and easy to maintain
- For 6-DOF and moderate iteration counts, it is fast enough in Python for your use case

---

### 3.3 Update step: Damped Least Squares (DLS)

We want to reduce the error:

$$
J\;\Delta q \approx e
$$

If \(J\) is well-conditioned, you could use least squares. But near singularities or poor conditioning, least squares becomes unstable.

So the node uses **Damped Least Squares**:

$$
\Delta q = J^T (J J^T + \lambda^2 I)^{-1}\; e
$$

This matches your code:

```python
JJt = J @ J.T
dq_step = J.T @ solve(JJt + lam**2 * I, e)
q = q + alpha * dq_step
```

**Meaning of each term:**
- \(JJ^T\) is always \(6\times 6\) (easy to invert)
- \(\lambda\) is the damping (your parameter `damping`)
- adding \(\lambda^2 I\) prevents blow-ups near singularities and makes the inverse numerically stable
- `alpha` is a step size (your parameter `alpha`) to avoid overshooting

---

### 3.4 Convergence criteria (exactly like the node)

The node stops when both:

$$
\|e_p\| < \text{tol\_pos}
\quad\text{and}\quad
\|e_\omega\| < \text{tol\_rot}
$$

These are your parameters `tol_pos` and `tol_rot`.

If it does not converge within `max_iters`, the node returns `ok=False` but still sends the “best effort” \(q\) it reached.

---

## 4) Special case you mentioned: set \(L_4=L_5=L_6=0\) (no tool)

If you modify the URDF so that:

$$
L_4=L_5=L_6=0
$$

then the FK simplifies to:

$$
T = T_z(b_z)\;R_z(q_1)T_z(L_1)\;R_y(q_2)T_z(L_2)\;R_y(q_3)T_z(L_3)\;R_z(q_4)\;R_y(q_5)\;R_z(q_6)
$$

- The last three joints become a **pure rotation block** (no extra translation).
- This gets closer to a “spherical wrist style” structure (because all translations are in the first 3 links),
  but whether it is a true spherical wrist still depends on your axis intersection geometry.

**Practical consequence:**
- The numeric IK continues to work unchanged (and often converges faster because there is less coupling).
- If you later want an analytic split, you would compute \(q_1,q_2,q_3\) from position and \(q_4,q_5,q_6\) from orientation, but only if the geometry matches the standard assumptions.

---

## 5) Summary (what to use when)

- If you keep a general toy geometry (offsets anywhere):  
  **Use the numeric IK in the node** (FK + error + finite-diff Jacobian + DLS)

- If you redesign the last three joints into a clean spherical wrist and define a tool offset:  
  You can implement **analytic IK** with:
  - wrist-center position for \(q_1,q_2,q_3\)
  - \(R_{36}\) extraction for \(q_4,q_5,q_6\)

For your current workflow (URDF changing often), the numeric method is the most robust and matches exactly what your running node does.

