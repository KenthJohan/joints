#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <cglm/cglm.h>
#include <flecs.h>
#include "sim_types.h"

#define MAX_CONSTRAINT_ROWS 4096
#define MAX_SOLVER_ITERS 64
#define MAX_REVOLUTE_TARGETS 16

typedef struct {
	ecs_entity_t body;
	vec2 anchor;
	vec2 jv;
	double jw;
	double inv_mass;
	double inv_inertia;
} BodyConstraintTerm;

typedef struct {
	double bias;
	double alpha;
	double effective_mass;
	double lambda;
	int solver_iters;
} SolverData;

typedef struct {
	BodyConstraintTerm a;
	BodyConstraintTerm b;
	SolverData solver;
} ConstraintRow;

typedef struct {
	ConstraintRow rows[MAX_CONSTRAINT_ROWS];
	int row_count;
	double iter_residual[MAX_SOLVER_ITERS];
	int iter_count;
} SolverFrameCache;

static SolverFrameCache g_solver_cache = {0};
static SolverFrameCache g_prev_solver_cache = {0};
static uint64_t g_frame_counter = 0;
static uint64_t g_assembly_frame_marker = UINT64_MAX;
static int g_dbg_joint_total = 0;
static int g_dbg_missing_target = 0;
static int g_dbg_missing_body = 0;
static int g_dbg_missing_revolute = 0;
static int g_dbg_missing_data = 0;
static int g_dbg_printed_missing_data = 0;

static double find_cached_lambda(
	const BodyConstraintTerm *a,
	const BodyConstraintTerm *b)
{
	for (int row_index = 0; row_index < g_prev_solver_cache.row_count; row_index ++) {
		const ConstraintRow *cached = &g_prev_solver_cache.rows[row_index];
		if (cached->a.body != a->body || cached->b.body != b->body) {
			continue;
		}
		if (cached->a.jv[0] == a->jv[0] && cached->a.jv[1] == a->jv[1] &&
			cached->b.jv[0] == b->jv[0] && cached->b.jv[1] == b->jv[1]) {
			return cached->solver.lambda;
		}
	}

	return 0.0;
}

static double inv_if_nonzero(double value)
{
	if (value == 0.0) {
		return 0.0;
	}
	return 1.0 / value;
}

static ecs_entity_t resolve_instance_pivot(
	const ecs_world_t *ecs,
	ecs_entity_t source_pivot,
	ecs_entity_t base_pivot)
{
	if (base_pivot == 0) {
		return 0;
	}

	const ecs_entity_t base_body = ecs_get_target(ecs, base_pivot, EcsChildOf, 0);
	const char *body_name = ecs_get_name(ecs, base_body);
	const char *pivot_name = ecs_get_name(ecs, base_pivot);
	if (base_body == 0 || body_name == NULL || pivot_name == NULL) {
		return 0;
	}

	const ecs_entity_t source_body = ecs_get_target(ecs, source_pivot, EcsChildOf, 0);
	const ecs_entity_t simulation_scope = ecs_get_target(ecs, source_body, EcsChildOf, 0);
	if (simulation_scope == 0) {
		return 0;
	}

	const ecs_entity_t instance_body = ecs_lookup_child(ecs, simulation_scope, body_name);
	if (instance_body == 0) {
		return 0;
	}

	return ecs_lookup_child(ecs, instance_body, pivot_name);
}

/* Return -1 when out of row capacity, 0 when pair cannot be assembled, 1 on success. */
static int append_revolute_pair_rows(
	ecs_world_t *ecs,
	ecs_entity_t pivot_a,
	ecs_entity_t pivot_b,
	double beta,
	double dt,
	double alpha,
	int row_solver_iters)
{
	if (g_solver_cache.row_count + 2 > MAX_CONSTRAINT_ROWS) {
		return -1;
	}

	const ecs_entity_t body_a = ecs_get_target(ecs, pivot_a, EcsChildOf, 0);
	const ecs_entity_t body_b = ecs_get_target(ecs, pivot_b, EcsChildOf, 0);
	if (body_a == 0 || body_b == 0) {
		g_dbg_missing_body ++;
		return 0;
	}

	const Pose *pose_a = ecs_get(ecs, body_a, Pose);
	const Pose *pose_b = ecs_get(ecs, body_b, Pose);
	const Mass *mass_a = ecs_get(ecs, body_a, Mass);
	const Mass *mass_b = ecs_get(ecs, body_b, Mass);
	const Inertia *inertia_a = ecs_get(ecs, body_a, Inertia);
	const Inertia *inertia_b = ecs_get(ecs, body_b, Inertia);
	const LocalOffset *local_a = ecs_get(ecs, pivot_a, LocalOffset);
	const LocalOffset *local_b = ecs_get(ecs, pivot_b, LocalOffset);

	if (pose_a == NULL || pose_b == NULL || mass_a == NULL || mass_b == NULL ||
		inertia_a == NULL || inertia_b == NULL || local_a == NULL || local_b == NULL) {
		g_dbg_missing_data ++;
		if (!g_dbg_printed_missing_data) {
			g_dbg_printed_missing_data = 1;
			printf("assembly missing data joint=%s pose_a=%d pose_b=%d mass_a=%d mass_b=%d inertia_a=%d inertia_b=%d local_a=%d local_b=%d\n",
				ecs_get_name(ecs, pivot_a),
				pose_a != NULL,
				pose_b != NULL,
				mass_a != NULL,
				mass_b != NULL,
				inertia_a != NULL,
				inertia_b != NULL,
				local_a != NULL,
				local_b != NULL);
		}
		return 0;
	}

	vec2 local_av = {(float)local_a->x, (float)local_a->y};
	vec2 local_bv = {(float)local_b->x, (float)local_b->y};
	vec2 ra;
	vec2 rb;
	glm_vec2_rotate(local_av, (float)pose_a->angle, ra);
	glm_vec2_rotate(local_bv, (float)pose_b->angle, rb);

	vec2 pa = {(float)pose_a->x, (float)pose_a->y};
	vec2 pb = {(float)pose_b->x, (float)pose_b->y};
	vec2 anchor_a;
	vec2 anchor_b;
	glm_vec2_add(pa, ra, anchor_a);
	glm_vec2_add(pb, rb, anchor_b);

	vec2 delta_anchor;
	glm_vec2_sub(anchor_b, anchor_a, delta_anchor);

	vec2 axes[2] = {
		{1.0f, 0.0f},
		{0.0f, 1.0f}
	};

	for (int row_axis = 0; row_axis < 2; row_axis ++) {
		ConstraintRow *row = &g_solver_cache.rows[g_solver_cache.row_count ++];
		vec2 axis;
		glm_vec2_copy(axes[row_axis], axis);

		row->a.body = body_a;
		row->b.body = body_b;
		glm_vec2_copy(anchor_a, row->a.anchor);
		glm_vec2_copy(anchor_b, row->b.anchor);

		// Sequential Impulse row assembly: n is the row direction (constraint axis) in world space.
		// A uses -n and B uses +n so the row enforces relative motion along n with equal-and-opposite impulses.
		glm_vec2_negate_to(axis, row->a.jv);
		glm_vec2_copy(axis, row->b.jv);
		
		// Sequential Impulse row assembly: angular Jacobian for body A (Jw = -(r x n)).
		row->a.jw = -(double)glm_vec2_cross(ra, axis);
		// Sequential Impulse row assembly: angular Jacobian for body B (Jw = r x n).
		row->b.jw = (double)glm_vec2_cross(rb, axis);
		// With unit row axes here (x/y basis), |a.jv|^2 = a.jv.x^2 + a.jv.y^2 = 1
		// and |b.jv|^2 = b.jv.x^2 + b.jv.y^2 = 1, so linear terms simplify to
		// a.inv_mass and b.inv_mass (no explicit jv-square factors needed).

		row->a.inv_mass = inv_if_nonzero(mass_a->value);
		row->b.inv_mass = inv_if_nonzero(mass_b->value);
		row->a.inv_inertia = inv_if_nonzero(inertia_a->value);
		row->b.inv_inertia = inv_if_nonzero(inertia_b->value);
		row->solver.bias = (beta / dt) * (double)glm_vec2_dot(delta_anchor, axis);
		row->solver.alpha = alpha;
		row->solver.lambda = find_cached_lambda(&row->a, &row->b);
		row->solver.solver_iters = row_solver_iters;

		// Per-row denominator: k = J M^-1 J^T + alpha (compliance regularization).
		// J is a 1x6 row vector (Jacobian): [a.jv.x, a.jv.y, a.jw, b.jv.x, b.jv.y, b.jw].
		// M is a 6x6 diagonal matrix (generalized mass): diag(m_a, m_a, I_a, m_b, m_b, I_b).
		// M^-1 is a 6x6 diagonal matrix using [a.inv_mass, a.inv_mass, a.inv_inertia,
		//                                     b.inv_mass, b.inv_mass, b.inv_inertia].
		// J^T is a 6x1 column vector, alpha is a scalar, and k is a scalar.
		const double k =
			row->a.inv_mass + row->b.inv_mass +
			(row->a.jw * row->a.jw) * row->a.inv_inertia +
			(row->b.jw * row->b.jw) * row->b.inv_inertia +
			row->solver.alpha;

		if (k <= 1e-9) {
			g_solver_cache.row_count --;
			continue;
		}

		row->solver.effective_mass = k;
	}

	return 1;
}

static void apply_impulse_delta(ecs_world_t *ecs, const ConstraintRow *row, double delta_lambda)
{
	if (delta_lambda == 0.0) {
		return;
	}

	Velocity *vel_a = ecs_get_mut(ecs, row->a.body, Velocity);
	if (vel_a != NULL) {
		vec2 delta_va = {
			(float)(row->a.inv_mass * row->a.jv[0] * delta_lambda),
			(float)(row->a.inv_mass * row->a.jv[1] * delta_lambda)
		};
		vel_a->x += (double)delta_va[0];
		vel_a->y += (double)delta_va[1];
		vel_a->angular += row->a.inv_inertia * row->a.jw * delta_lambda;
		ecs_modified(ecs, row->a.body, Velocity);
	}

	Velocity *vel_b = ecs_get_mut(ecs, row->b.body, Velocity);
	if (vel_b != NULL) {
		vec2 delta_vb = {
			(float)(row->b.inv_mass * row->b.jv[0] * delta_lambda),
			(float)(row->b.inv_mass * row->b.jv[1] * delta_lambda)
		};
		vel_b->x += (double)delta_vb[0];
		vel_b->y += (double)delta_vb[1];
		vel_b->angular += row->b.inv_inertia * row->b.jw * delta_lambda;
		ecs_modified(ecs, row->b.body, Velocity);
	}
}

static double compute_row_residual(const ecs_world_t *ecs, const ConstraintRow *row)
{
	const Velocity *vel_a = ecs_get(ecs, row->a.body, Velocity);
	const Velocity *vel_b = ecs_get(ecs, row->b.body, Velocity);

	double jv = 0.0;
	if (vel_a != NULL) {
		vec2 jva = {row->a.jv[0], row->a.jv[1]};
		vec2 va = {(float)vel_a->x, (float)vel_a->y};
		jv += (double)glm_vec2_dot(jva, va) + row->a.jw * vel_a->angular;
	}
	if (vel_b != NULL) {
		vec2 jvb = {row->b.jv[0], row->b.jv[1]};
		vec2 vb = {(float)vel_b->x, (float)vel_b->y};
		jv += (double)glm_vec2_dot(jvb, vb) + row->b.jw * vel_b->angular;
	}

	return jv + row->solver.bias + row->solver.alpha * row->solver.lambda;
}

void AssembleRevoluteRows(ecs_iter_t *it)
{
	assert(it->field_count >= 3);
	const SolverConfig *solver_cfg = ecs_field(it, SolverConfig, 2); // shared

	assert(solver_cfg != NULL);

	if (g_assembly_frame_marker != g_frame_counter) {
		g_assembly_frame_marker = g_frame_counter;
		g_solver_cache.row_count = 0;
		g_solver_cache.iter_count = 0;
		g_dbg_joint_total = 0;
		g_dbg_missing_target = 0;
		g_dbg_missing_body = 0;
		g_dbg_missing_revolute = 0;
		g_dbg_missing_data = 0;
		g_dbg_printed_missing_data = 0;
	}

	for (int i = 0; i < it->count; i ++) {
		g_dbg_joint_total ++;

		const ecs_entity_t pivot_a = it->entities[i];
		const ecs_entity_t base_pivot_a = ecs_get_target(it->world, pivot_a, EcsIsA, 0);

		const double dt = (solver_cfg->dt > 0.0) ? solver_cfg->dt : ((it->delta_time > 0.0) ? (double)it->delta_time : (1.0 / 60.0));
		const double beta = solver_cfg->baumgarte;
		const int configured_iters = (solver_cfg->iterations > 0) ? solver_cfg->iterations : 10;
		const int row_solver_iters = configured_iters > MAX_SOLVER_ITERS ? MAX_SOLVER_ITERS : configured_iters;
		const double compliance_value = solver_cfg->compliance;
		const double alpha = (compliance_value > 0.0) ? (compliance_value / (dt * dt)) : 0.0;
		int appended_any = 0;

		for (int target_index = 0; target_index < MAX_REVOLUTE_TARGETS; target_index ++) {
			ecs_entity_t pivot_b = ecs_get_target(it->world, pivot_a, ecs_id(Revolute), target_index);
			if (pivot_b == 0 && base_pivot_a != 0) {
				pivot_b = ecs_get_target(it->world, base_pivot_a, ecs_id(Revolute), target_index);
			}
			if (pivot_b == 0) {
				break;
			}

			ecs_entity_t remapped = resolve_instance_pivot(it->world, pivot_a, pivot_b);
			if (remapped != 0) {
				pivot_b = remapped;
			}

			const int append_result = append_revolute_pair_rows(
				it->world,
				pivot_a,
				pivot_b,
				beta,
				dt,
				alpha,
				row_solver_iters);
			if (append_result < 0) {
				break;
			}
			if (append_result > 0) {
				appended_any = 1;
			}
		}

		if (!appended_any) {
			g_dbg_missing_target ++;
		}
	}
}

void SolveConstraintRows(ecs_iter_t *it)
{
	if (g_solver_cache.row_count == 0) {
		g_prev_solver_cache.row_count = 0;
		g_prev_solver_cache.iter_count = 0;
		printf("[frame %llu] global convergence no_rows joints=%d missing_target=%d missing_body=%d missing_revolute=%d missing_data=%d\n",
			(unsigned long long)g_frame_counter,
			g_dbg_joint_total,
			g_dbg_missing_target,
			g_dbg_missing_body,
			g_dbg_missing_revolute,
			g_dbg_missing_data);
		g_frame_counter ++;
		return;
	}

	int solver_iters = 0;
	for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
		if (g_solver_cache.rows[row_index].solver.solver_iters > solver_iters) {
			solver_iters = g_solver_cache.rows[row_index].solver.solver_iters;
		}
	}
	if (solver_iters <= 0) {
		solver_iters = 1;
	}

	for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
		apply_impulse_delta(it->world, &g_solver_cache.rows[row_index], g_solver_cache.rows[row_index].solver.lambda);
	}

	for (int iter = 0; iter < solver_iters; iter ++) {
		double global_sq = 0.0;

		for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
			ConstraintRow *row = &g_solver_cache.rows[row_index];
				if (iter >= row->solver.solver_iters) {
					continue;
				}
				const double residual = compute_row_residual(it->world, row);
				// Gauss-Seidel row update: delta_lambda = -(Jv + bias + alpha * lambda) / k.
				const double delta_lambda = -residual / row->solver.effective_mass;
				row->solver.lambda += delta_lambda;
			apply_impulse_delta(it->world, row, delta_lambda);

			const double post_residual = compute_row_residual(it->world, row);
			global_sq += post_residual * post_residual;
		}

		g_solver_cache.iter_residual[iter] = sqrt(global_sq);
	}

	g_solver_cache.iter_count = solver_iters;

	printf("[frame %llu] global convergence", (unsigned long long)g_frame_counter);
	for (int iter = 0; iter < g_solver_cache.iter_count; iter ++) {
		printf(" i%d=%.3e", iter, g_solver_cache.iter_residual[iter]);
	}
	printf("\n");
	g_prev_solver_cache = g_solver_cache;

	g_frame_counter ++;
}

void IntegrateBodies(ecs_iter_t *it)
{
	assert(it->field_count >= 3);
	const Velocity *velocity = ecs_field(it, Velocity, 0); // self
	Pose *pose = ecs_field(it, Pose, 1); // self
	const SolverConfig *solver_cfg = ecs_field(it, SolverConfig, 2); // shared

	assert(velocity != NULL);
	assert(pose != NULL);
	assert(solver_cfg != NULL);

	for (int i = 0; i < it->count; i ++) {
		const double dt = (solver_cfg->dt > 0.0) ? solver_cfg->dt : ((it->delta_time > 0.0) ? (double)it->delta_time : (1.0 / 60.0));
		vec2 position = {(float)pose[i].x, (float)pose[i].y};
		vec2 linear_velocity = {(float)velocity[i].x, (float)velocity[i].y};
		vec2 delta_position;

		glm_vec2_scale(linear_velocity, (float)dt, delta_position);
		glm_vec2_add(position, delta_position, position);

		pose[i].x = (double)position[0];
		pose[i].y = (double)position[1];
		pose[i].angle += velocity[i].angular * dt;
		ecs_modified(it->world, it->entities[i], Pose);
	}
}

void PrintRevolutePairs(ecs_iter_t *it)
{
	assert(it->field_count >= 1);

	const ecs_id_t pair_id = ecs_field_id(it, 0);
	if (!ECS_IS_PAIR(pair_id)) {
		return;
	}

	const ecs_entity_t body_b = ecs_pair_second(it->world, pair_id);
	const char *body_b_name = ecs_get_name(it->world, body_b);

	for (int i = 0; i < it->count; i ++) {
		const ecs_entity_t body_a = it->entities[i];
		const char *body_a_name = ecs_get_name(it->world, body_a);
		printf("%s-%s\n",
			(body_a_name != NULL) ? body_a_name : "<unnamed>",
			(body_b_name != NULL) ? body_b_name : "<unnamed>");
	}
}

int main(int argc, char *argv[])
{
	ecs_world_t *ecs = ecs_init_w_args(argc, argv);

	ECS_IMPORT(ecs, SimTypes);

	ecs_system_init(ecs, &(ecs_system_desc_t){
		.entity = ecs_entity(ecs, {.name = "AssembleRevoluteRows"}),
		.query.terms = {
			{.id = ecs_id(LocalOffset)},
			{.id = ecs_pair(ecs_id(Revolute), EcsWildcard)},
			{.id = ecs_id(SolverConfig), .src.id = EcsUp, .trav = EcsChildOf}
		},
		.callback = AssembleRevoluteRows,
		.phase = EcsPreUpdate
	});

	ecs_system_init(ecs, &(ecs_system_desc_t){
		.entity = ecs_entity(ecs, {.name = "SolveConstraintRows"}),
		.query.terms = {
			{.id = ecs_id(SolverConfig)}
		},
		.callback = SolveConstraintRows,
		.phase = EcsOnUpdate
	});

	ecs_system_init(ecs, &(ecs_system_desc_t){
		.entity = ecs_entity(ecs, {.name = "IntegrateBodies"}),
		.query.terms = {
			{.id = ecs_id(Velocity)},
			{.id = ecs_id(Pose)},
			{ .id = ecs_id(SolverConfig), .src.id = EcsUp, .trav = EcsChildOf }
		},
		.callback = IntegrateBodies,
		.phase = EcsPostUpdate
	});

	ecs_system_init(ecs, &(ecs_system_desc_t){
		.entity = ecs_entity(ecs, {.name = "PrintRevolutePairs"}),
		.query.terms = {
			{.id = ecs_pair(ecs_id(Revolute), EcsWildcard)}
		},
		.callback = PrintRevolutePairs,
		.phase = EcsOnStart
	});

	if (ecs_script_run_file(ecs, "assets/entities.flecs")) {
		return 1;
	}

	printf("startup counts: SolverConfig=%d\n",
		ecs_count_id(ecs, ecs_id(SolverConfig)));

	for (int frame = 0; frame < 240; frame ++) {
		if (!ecs_progress(ecs, (ecs_ftime_t)(1.0 / 60.0))) {
			break;
		}
	}

	// Used for remote inspection with Flecs Explorer. See https://www.flecs.dev/explorer/?remote=true
#if 0
	ecs_set(ecs, EcsWorld, EcsRest, {.port = 0});
	printf("Remote: %s\n", "https://www.flecs.dev/explorer/?remote=true");

	while (1) {
		ecs_progress(ecs, 1.0f);
	}
#endif

	return ecs_fini(ecs);
}
