# Method Sketch: Sequential Impulse Gauss-Seidel (Velocity-Level)

## Correct Method Name

The solver in [src/main.c](src/main.c) is best described as:

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

Relevant implementation points:

- Row assembly and Jacobian terms in [src/main.c](src/main.c#L175)
- Bias and compliance assignment in [src/main.c](src/main.c#L222)
- Per-row residual evaluation in [src/main.c](src/main.c#L267)
- Gauss-Seidel impulse update in [src/main.c](src/main.c#L385)
- Immediate velocity update after each row in [src/main.c](src/main.c#L244)

## Solver Form Used

For each scalar row, solve iteratively:

Jv + bias + alpha * lambda = 0

with update:

delta_lambda = -(Jv + bias + alpha * lambda) / k

where:

- k = J M^-1 J^T + alpha
- alpha is compliance-derived softening
- lambda is warm-started and iteratively refined

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
