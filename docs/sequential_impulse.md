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

So the per-body constraint-velocity contributions are:

$$
Jv_A = n \cdot (v_A + \omega_A r_A^\perp)
$$

$$
Jv_B = n \cdot (v_B + \omega_B r_B^\perp)
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

These are the angular terms that appear in both $Jv_A$/$Jv_B$ and the denominator contribution $J M^{-1} J^T$.

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

### How Rows Are Created From Revolute And Prismatic Constraints

The solver operates on scalar rows. A joint contributes one or more rows depending on which DOFs are constrained.

For each row, assembly follows the same pattern:

- choose a row direction (or angular mode)
- compute Jacobian terms for body A and body B
- compute $Jv_A$ and $Jv_B$
- compute positional/angular error for Baumgarte bias
- compute denominator $k = J M^{-1} J^T + \alpha$

#### Revolute Joint (2D)

A revolute joint enforces coincidence of two world-space anchor points:

$$
C = (x_A + r_A) - (x_B + r_B) = 0
$$

In 2D, this gives 2 scalar translational rows (x and y axes):

- Row 1: $n=(1,0)$
- Row 2: $n=(0,1)$

Each row uses:

$$
Jv_A = n \cdot (v_A + \omega_A r_A^\perp),
\qquad
Jv_B = n \cdot (v_B + \omega_B r_B^\perp)
$$

with residual form:

$$
r = Jv_A - Jv_B + \mathrm{bias} + \alpha\lambda
$$

So a revolute joint contributes exactly 2 equality rows in this 2D formulation.

#### Prismatic Joint (2D)

A prismatic joint allows relative translation along a chosen axis $t$ and constrains motion in the perpendicular direction $n=t^\perp$.

Typical equality rows are:

1. Lateral translation row (remove sideways drift):

$$
Jv_A^{\perp} = n \cdot (v_A + \omega_A r_A^\perp),
\qquad
Jv_B^{\perp} = n \cdot (v_B + \omega_B r_B^\perp)
$$

$$
r_{\perp} = Jv_A^{\perp} - Jv_B^{\perp} + \mathrm{bias}_{\perp} + \alpha_{\perp}\lambda_{\perp}
$$

2. Relative angle row (keep bodies aligned to the slide axis frame):

$$
r_{\theta} = (\omega_A - \omega_B) + \mathrm{bias}_{\theta} + \alpha_{\theta}\lambda_{\theta}
$$

So, for equality-only behavior, a prismatic joint usually contributes 2 rows in 2D:

- one linear row along $n$
- one angular row

Optional motor/limit behavior along axis $t$ is usually implemented as additional rows (or inequality rows) on top of these base rows.

### Joint Type Summary

#### Description Table

| Joint type                          | What is constrained                                                       | Rows needed in 2D           |
| ----------------------------------- | ------------------------------------------------------------------------- | --------------------------- |
| Revolute                            | Coincidence of the two anchor points                                      | 2 equality rows             |
| Prismatic                           | Relative translation perpendicular to the slide axis, plus relative angle | 2 equality rows             |
| Prismatic                           | Relative translation perpendicular to the slide axis, plus relative angle | 2 equality rows             |
| Distance                            | Separation along the line between two anchor points                       | 1 equality row              |
| Weld                                | Relative translation and relative angle                                   | 3 equality rows             |
| Weld                                | Relative translation and relative angle                                   | 3 equality rows             |
| Fixed rotation                      | Relative angle only                                                       | 1 equality row              |
| Motor on revolute axis              | Target relative angular speed                                             | Usually +1 row              |
| Motor on prismatic axis             | Target relative slide speed                                               | Usually +1 row              |
| Limit on revolute or prismatic axis | Relative angle or slide position bound                                    | 0 or 1 active row per limit |

#### Math Table

| Joint type                            | $n$                                                     | $J_{vA}$              | $J_{vB}$              | $J_{\omega A}$        | $J_{\omega B}$        |
| ------------------------------------- | ------------------------------------------------------- | --------------------- | --------------------- | --------------------- | --------------------- |
| Revolute 0                            | $(1,0)$                                                 | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Revolute 1                            | $(0,1)$                                                 | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Prismatic 0                           | $t^\perp$                                              | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Prismatic 1                           | $-$                                                     | $0$                   | $0$                   | $-1$                  | $1$                   |
| Distance 0                            | $n$                                                     | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Weld 0                                | $(1,0)$                                                 | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Weld 1                                | $(0,1)$                                                 | $-n$                  | $n$                   | $-(r_A \times n)$     | $r_B \times n$        |
| Weld 2                                | $-$                                                     | $0$                   | $0$                   | $-1$                  | $1$                   |
| Fixed rotation 0                      | $-$                                                     | $0$                   | $0$                   | $-1$                  | $1$                   |
| Motor on revolute axis 0              | $-$                                                     | $0$                   | $0$                   | $-1$                  | $1$                   |
| Motor on prismatic axis 0             | $t$                                                     | $-t$                  | $t$                   | $-(r_A \times t)$     | $r_B \times t$        |
| Limit on revolute or prismatic axis 0 | $n_{\mathrm{limit}}$                                   | Same as limited DOF   | Same as limited DOF   | Same as limited DOF   | Same as limited DOF   |

### Variables Used In This Section

| Symbol                | Type                  | Meaning                                                                                                                             |
| --------------------- | --------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| $\alpha$              | scalar                | Compliance regularization (CFM-like softening) term added to the row equation and denominator.                                      |
| $J$                   | row matrix (Jacobian) | Full constraint Jacobian row: $[J_{vAx},\ J_{vAy},\ J_{\omega A},\ J_{vBx},\ J_{vBy},\ J_{\omega B}]$.                              |
| $J_{\omega A}$        | scalar                | Body A angular Jacobian component, $J_{\omega A}=r_A \times n$.                                                                     |
| $J_{\omega B}$        | scalar                | Body B angular Jacobian component, $J_{\omega B}=-(r_B \times n)$.                                                                  |
| $Jv_A$                | scalar                | Body A contribution to row velocity, $Jv_A = n \cdot (v_A + \omega_A r_A^\perp)$.                                                   |
| $Jv_B$                | scalar                | Body B contribution to row velocity, $Jv_B = n \cdot (v_B + \omega_B r_B^\perp)$.                                                   |
| $k$                   | scalar                | Effective row denominator, $k = J M^{-1} J^T + \alpha$, used to scale impulse updates.                                              |
| $M$                   | matrix                | Generalized mass matrix for the bodies in a constraint row; $M^{-1}$ is the generalized inverse mass matrix used in $J M^{-1} J^T$. |
| $n$                   | 2D unit vector        | Constraint row direction in world space (for example x-axis or y-axis).                                                             |
| $r$                   | 2D vector             | Offset from body center of mass to the constraint point in world space.                                                             |
| $r^\perp$             | 2D vector             | Perpendicular vector to $r$, defined as $r^\perp = (-r_y,\ r_x)$.                                                                   |
| $r_A,\ r_B$           | 2D vectors            | Point offsets for body A and body B, respectively ($r_A, r_B$).                                                                     |
| $r \times n$          | scalar                | 2D scalar cross product $r \times n = r_x n_y - r_y n_x = n \cdot r^\perp$.                                                         |
| $v_A,\ v_B$           | 2D vectors            | Linear velocities of body A and body B, respectively ($v_A, v_B$).                                                                  |
| $v_p$                 | 2D vector             | Velocity of a constraint point on a body in world space ($v_p$).                                                                    |
| $\omega$              | scalar                | Angular velocity in 2D (about out-of-plane z-axis).                                                                                 |
| $\omega_A,\ \omega_B$ | scalars               | Angular velocities of body A and body B, respectively ($\omega_A, \omega_B$).                                                       |



## Solver Form Used

For each scalar row, solve iteratively:

$$
Jv_A - Jv_B + \mathrm{bias} + \alpha \lambda = 0
$$

with update:

$$
\Delta \lambda = -\frac{Jv_A - Jv_B + \mathrm{bias} + \alpha \lambda}{k}
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
        residual = JvA(row) - JvB(row) + bias(row) + alpha(row) * lambda(row)
        delta = -residual / effective_mass(row)
        lambda(row) += delta
        apply_impulse_delta(row, delta)

        post = JvA(row) - JvB(row) + bias(row) + alpha(row) * lambda(row)
        global_sq += post * post

    residual_trace[iter] = sqrt(global_sq)
```

## Notes On Terminology

- Calling it Projected Gauss-Seidel is common in game physics literature.
- In this specific implementation, equality rows are not clamped, so projection is not the central feature.
- The most accurate neutral name here is Sequential Impulse Gauss-Seidel for velocity constraints.
