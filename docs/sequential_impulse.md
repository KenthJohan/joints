# Method Sketch: Sequential Impulse Gauss-Seidel (Velocity-Level)

## Correct Method Name

The solver is best described as:

- Sequential Impulse Gauss-Seidel at velocity level for equality constraints
- with Baumgarte stabilization (bias term)
- with optional compliance regularization (CFM-like softening)
- with warm starting

Short name:

- Sequential Impulse (SI) / Gauss-Seidel constraint solver

## Why This Name Matches The Code

- Rows are assembled per constraint direction and solved one row at a time.
- Each row update uses current velocities and immediately writes impulses back to bodies.
- The solve loop repeats this row-by-row process for multiple iterations.
- A Baumgarte bias term drives positional error reduction through velocity constraints.
- Compliance enters as an additive regularization term in the row denominator/equation.

## What The Denominator Block Is

During row assembly, each scalar row computes a denominator term that scales the impulse update:

$$
k = J M^{-1} J^T + \alpha
$$

This term belongs to the row precompute stage and is reused during each Gauss-Seidel update for that row.

### Why The Linear Part Simplifies

The translational rows use unit world axes (x and y). With unit row directions, the linear Jacobian magnitudes are 1, so the translational contribution reduces to inverse masses of the two bodies.

The rotational contribution remains explicit through angular Jacobian terms and inverse inertias. Compliance is added through $\alpha$ as regularization.

### How The Angular Jacobian Is Calculated (2D)

For a scalar velocity constraint along world direction $n$, the point velocity on each body is:

$$
v_p = v + \omega \times r
$$

In 2D, angular velocity is scalar and:

$$
\omega \times r = \omega\, r^\perp,
\qquad
r^\perp = (-r_y,\ r_x)
$$

So the constraint velocity is:

$$
Jv = n \cdot (v_A + \omega_A r_A^\perp) - n \cdot (v_B + \omega_B r_B^\perp)
$$

Collecting coefficients of $\omega_A$ and $\omega_B$ gives the angular Jacobian entries:

$$
J_{\omega A} = n \cdot r_A^\perp,
\qquad
J_{\omega B} = -\,n \cdot r_B^\perp
$$

Equivalent scalar form (2D cross product):

$$
n \cdot r^\perp = r_x n_y - r_y n_x = r \times n
$$

Therefore:

$$
J_{\omega A} = r_A \times n,
\qquad
J_{\omega B} = -(r_B \times n)
$$

For axis rows used in this solver:

$$
n = (1,0) \Rightarrow J_{\omega} = -r_y,
\qquad
n = (0,1) \Rightarrow J_{\omega} = r_x
$$

These are the angular terms that appear in both $Jv$ and the denominator contribution $J M^{-1} J^T$.

### What Components $J$ Consists Of

For one scalar row in a 2D two-body constraint, use generalized velocity ordering:

$$
qdot = [v_{Ax},\ v_{Ay},\ \omega_A,\ v_{Bx},\ v_{By},\ \omega_B]^T
$$

Then the Jacobian row is:

$$
J = [J_{vAx},\ J_{vAy},\ J_{\omega A},\ J_{vBx},\ J_{vBy},\ J_{\omega B}]
$$

with linear components:

$$
J_{vA} = n, \qquad J_{vB} = -n
$$

and angular components:

$$
J_{\omega A} = r_A \times n, \qquad J_{\omega B} = -(r_B \times n)
$$

So an explicit row form is:

$$
J = [n_x,\ n_y,\ r_A \times n,\ -n_x,\ -n_y,\ -(r_B \times n)]
$$

Axis-row examples used here:

$$
n=(1,0) \Rightarrow J=[1,0,-r_{Ay},-1,0,r_{By}]
$$

$$
n=(0,1) \Rightarrow J=[0,1,r_{Ax},0,-1,-r_{Bx}]
$$

### Variables Used In This Section

| Base Symbol | Full Symbol Form | Type | Meaning |
|---|---|---|---|
| $\alpha$ | $\alpha$ | scalar | Compliance regularization (CFM-like softening) term added to the row equation and denominator. |
| $J$ | $J$ | row matrix (Jacobian) | Full constraint Jacobian row: $[J_{vAx},\ J_{vAy},\ J_{\omega A},\ J_{vBx},\ J_{vBy},\ J_{\omega B}]$. |
| $J$ | $J_{vAx}$ | scalar | Body A linear Jacobian x component, $J_{vAx}=n_x$. |
| $J$ | $J_{vAy}$ | scalar | Body A linear Jacobian y component, $J_{vAy}=n_y$. |
| $J$ | $J_{vBx}$ | scalar | Body B linear Jacobian x component, $J_{vBx}=-n_x$. |
| $J$ | $J_{vBy}$ | scalar | Body B linear Jacobian y component, $J_{vBy}=-n_y$. |
| $J$ | $J_{\omega A}$ | scalar | Body A angular Jacobian component, $J_{\omega A}=r_A \times n$. |
| $J$ | $J_{\omega B}$ | scalar | Body B angular Jacobian component, $J_{\omega B}=-(r_B \times n)$. |
| $v$ | $Jv$ | scalar | Relative row velocity $Jv$ after applying the Jacobian to generalized velocities. |
| $k$ | $k$ | scalar | Effective row denominator, $k = J M^{-1} J^T + \alpha$, used to scale impulse updates. |
| $M$ | $M$ | matrix | Generalized mass matrix for the bodies in a constraint row; $M^{-1}$ is the generalized inverse mass matrix used in $J M^{-1} J^T$. |
| $n$ | $n$ | 2D unit vector | Constraint row direction in world space (for example x-axis or y-axis). |
| $r$ | $r$ | 2D vector | Offset from body center of mass to the constraint point in world space. |
| $r$ | $r^\perp$ | 2D vector | Perpendicular vector to $r$, defined as $r^\perp = (-r_y,\ r_x)$. |
| $r$ | $r_A,\ r_B$ | 2D vectors | Point offsets for body A and body B, respectively ($r_A, r_B$). |
| $r$ | $r \times n$ | scalar | 2D scalar cross product $r \times n = r_x n_y - r_y n_x = n \cdot r^\perp$. |
| $v$ | $v_A,\ v_B$ | 2D vectors | Linear velocities of body A and body B, respectively ($v_A, v_B$). |
| $v$ | $v_{\mathrm{com}}$ | 2D vector | Linear velocity of the body center of mass in world space. |
| $v$ | $v_p$ | 2D vector | Velocity of a constraint point on a body in world space ($v_p$). |
| $\omega$ | $\omega$ | scalar | Angular velocity in 2D (about out-of-plane z-axis). |
| $\omega$ | $\omega_A,\ \omega_B$ | scalars | Angular velocities of body A and body B, respectively ($\omega_A, \omega_B$). |

Type notes:

- 2D vector values have components $(x,y)$.
- Scalars are single real numbers.

## Solver Form Used

For each scalar row, solve iteratively:

$$
Jv + \mathrm{bias} + \alpha \lambda = 0
$$

with update:

$$
\Delta \lambda = -\frac{Jv + \mathrm{bias} + \alpha \lambda}{k}
$$

where:

- $k = J M^{-1} J^T + \alpha$
- $\alpha$ is the compliance regularization (CFM-like softening) term
- $\lambda$ is warm-started and iteratively refined

Interpretation:

- The denominator $k$ is the effective row stiffness at velocity level.
- Larger $k$ gives smaller impulse changes per iteration.
- Positive $\alpha$ softens constraints and improves numerical robustness.

## Pseudocode Sketch

```text
assemble_rows()

for each row:
    apply_warm_start_impulse(row.lambda)

for iter in 0..solver_iters-1:
    global_sq = 0

    for each row:
        residual = Jv(row) + bias(row) + alpha(row) * lambda(row)
        delta = -residual / effective_mass(row)
        lambda(row) += delta
        apply_impulse_delta(row, delta)

        post = Jv(row) + bias(row) + alpha(row) * lambda(row)
        global_sq += post * post

    residual_trace[iter] = sqrt(global_sq)
```

## Notes On Terminology

- Calling it Projected Gauss-Seidel is common in game physics literature.
- In this specific implementation, equality rows are not clamped, so projection is not the central feature.
- The most accurate neutral name here is Sequential Impulse Gauss-Seidel for velocity constraints.
