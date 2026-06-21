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

## Solver Form Used

For each scalar row, solve iteratively:

Jv + bias + alpha * lambda = 0

with update:

delta_lambda = -(Jv + bias + alpha * lambda) / k

where:

- k = J M^-1 J^T + alpha
- alpha is compliance-derived softening
- lambda is warm-started and iteratively refined

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
