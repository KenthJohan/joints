#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <flecs.h>
#include "sim_types.h"

#define MAX_CONSTRAINT_ROWS 4096
#define MAX_SOLVER_ITERS 64
#define MAX_REVOLUTE_TARGETS 16

typedef struct {
	ecs_entity_t joint;
	ecs_entity_t body_a;
	ecs_entity_t body_b;
	double anchor_ax;
	double anchor_ay;
	double anchor_bx;
	double anchor_by;
	double jv_ax;
	double jv_ay;
	double jw_a;
	double jv_bx;
	double jv_by;
	double jw_b;
	double inv_mass_a;
	double inv_mass_b;
	double inv_inertia_a;
	double inv_inertia_b;
	double bias;
	double alpha;
	double effective_mass;
	double lambda;
	int solver_iters;
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
	ecs_entity_t joint,
	ecs_entity_t body_a,
	ecs_entity_t body_b,
	double jv_ax,
	double jv_ay,
	double jv_bx,
	double jv_by)
{
	for (int row_index = 0; row_index < g_prev_solver_cache.row_count; row_index ++) {
		const ConstraintRow *cached = &g_prev_solver_cache.rows[row_index];
		if (cached->joint != joint || cached->body_a != body_a || cached->body_b != body_b) {
			continue;
		}
		if (cached->jv_ax == jv_ax && cached->jv_ay == jv_ay &&
			cached->jv_bx == jv_bx && cached->jv_by == jv_by) {
			return cached->lambda;
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

static void rotate_local(double x, double y, double angle, double *out_x, double *out_y)
{
	const double c = cos(angle);
	const double s = sin(angle);
	*out_x = c * x - s * y;
	*out_y = s * x + c * y;
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
	ecs_entity_t source_pivot,
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
				ecs_get_name(ecs, source_pivot),
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

	double ra_x, ra_y;
	double rb_x, rb_y;
	rotate_local(local_a->x, local_a->y, pose_a->angle, &ra_x, &ra_y);
	rotate_local(local_b->x, local_b->y, pose_b->angle, &rb_x, &rb_y);

	const double anchor_ax = pose_a->x + ra_x;
	const double anchor_ay = pose_a->y + ra_y;
	const double anchor_bx = pose_b->x + rb_x;
	const double anchor_by = pose_b->y + rb_y;

	const double dx = anchor_bx - anchor_ax;
	const double dy = anchor_by - anchor_ay;

	const double axes[2][2] = {
		{1.0, 0.0},
		{0.0, 1.0}
	};

	for (int row_axis = 0; row_axis < 2; row_axis ++) {
		ConstraintRow *row = &g_solver_cache.rows[g_solver_cache.row_count ++];
		const double nx = axes[row_axis][0];
		const double ny = axes[row_axis][1];

		row->joint = source_pivot;
		row->body_a = body_a;
		row->body_b = body_b;
		row->anchor_ax = anchor_ax;
		row->anchor_ay = anchor_ay;
		row->anchor_bx = anchor_bx;
		row->anchor_by = anchor_by;

		row->jv_ax = -nx;
		row->jv_ay = -ny;
		row->jw_a = -(ra_x * ny - ra_y * nx);
		row->jv_bx = nx;
		row->jv_by = ny;
		row->jw_b = rb_x * ny - rb_y * nx;

		row->inv_mass_a = inv_if_nonzero(mass_a->value);
		row->inv_mass_b = inv_if_nonzero(mass_b->value);
		row->inv_inertia_a = inv_if_nonzero(inertia_a->value);
		row->inv_inertia_b = inv_if_nonzero(inertia_b->value);
		row->bias = (beta / dt) * (dx * nx + dy * ny);
		row->alpha = alpha;
		row->lambda = find_cached_lambda(
			source_pivot,
			body_a,
			body_b,
			row->jv_ax,
			row->jv_ay,
			row->jv_bx,
			row->jv_by);
		row->solver_iters = row_solver_iters;

		const double k =
			row->inv_mass_a + row->inv_mass_b +
			(row->jw_a * row->jw_a) * row->inv_inertia_a +
			(row->jw_b * row->jw_b) * row->inv_inertia_b +
			row->alpha;

		if (k <= 1e-9) {
			g_solver_cache.row_count --;
			continue;
		}

		row->effective_mass = k;
	}

	return 1;
}

static void apply_impulse_delta(ecs_world_t *ecs, const ConstraintRow *row, double delta_lambda)
{
	if (delta_lambda == 0.0) {
		return;
	}

	Velocity *vel_a = ecs_get_mut(ecs, row->body_a, Velocity);
	if (vel_a != NULL) {
		vel_a->x += row->inv_mass_a * row->jv_ax * delta_lambda;
		vel_a->y += row->inv_mass_a * row->jv_ay * delta_lambda;
		vel_a->angular += row->inv_inertia_a * row->jw_a * delta_lambda;
		ecs_modified(ecs, row->body_a, Velocity);
	}

	Velocity *vel_b = ecs_get_mut(ecs, row->body_b, Velocity);
	if (vel_b != NULL) {
		vel_b->x += row->inv_mass_b * row->jv_bx * delta_lambda;
		vel_b->y += row->inv_mass_b * row->jv_by * delta_lambda;
		vel_b->angular += row->inv_inertia_b * row->jw_b * delta_lambda;
		ecs_modified(ecs, row->body_b, Velocity);
	}
}

static double compute_row_residual(const ecs_world_t *ecs, const ConstraintRow *row)
{
	const Velocity *vel_a = ecs_get(ecs, row->body_a, Velocity);
	const Velocity *vel_b = ecs_get(ecs, row->body_b, Velocity);

	double jv = 0.0;
	if (vel_a != NULL) {
		jv += row->jv_ax * vel_a->x + row->jv_ay * vel_a->y + row->jw_a * vel_a->angular;
	}
	if (vel_b != NULL) {
		jv += row->jv_bx * vel_b->x + row->jv_by * vel_b->y + row->jw_b * vel_b->angular;
	}

	return jv + row->bias + row->alpha * row->lambda;
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
		if (g_solver_cache.rows[row_index].solver_iters > solver_iters) {
			solver_iters = g_solver_cache.rows[row_index].solver_iters;
		}
	}
	if (solver_iters <= 0) {
		solver_iters = 1;
	}

	for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
		apply_impulse_delta(it->world, &g_solver_cache.rows[row_index], g_solver_cache.rows[row_index].lambda);
	}

	for (int iter = 0; iter < solver_iters; iter ++) {
		double global_sq = 0.0;

		for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
			ConstraintRow *row = &g_solver_cache.rows[row_index];
			if (iter >= row->solver_iters) {
				continue;
			}
			const double residual = compute_row_residual(it->world, row);
			const double delta_lambda = -residual / row->effective_mass;
			row->lambda += delta_lambda;
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

		pose[i].x += velocity[i].x * dt;
		pose[i].y += velocity[i].y * dt;
		pose[i].angle += velocity[i].angular * dt;
		ecs_modified(it->world, it->entities[i], Pose);
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
