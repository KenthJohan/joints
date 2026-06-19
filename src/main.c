#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <flecs.h>
#include "example_gsolver/sim_types.h"

#define MAX_CONSTRAINT_ROWS 4096
#define MAX_SOLVER_JOINTS 2048
#define MAX_SOLVER_ITERS 64

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
	int joint_slot;
} ConstraintRow;

typedef struct {
	ecs_entity_t joint;
	double residual_sq;
	double lambda_sq;
} JointResidual;

typedef struct {
	ConstraintRow rows[MAX_CONSTRAINT_ROWS];
	int row_count;
	JointResidual joints[MAX_SOLVER_JOINTS];
	int joint_count;
	double iter_residual[MAX_SOLVER_ITERS];
	int iter_count;
} SolverFrameCache;

static SolverFrameCache g_solver_cache = {0};
static uint64_t g_frame_counter = 0;
static uint64_t g_assembly_frame_marker = UINT64_MAX;
static ecs_entity_t g_solver_config_entity = 0;
static int g_dbg_joint_total = 0;
static int g_dbg_missing_mate = 0;
static int g_dbg_missing_body = 0;
static int g_dbg_missing_revolute = 0;
static int g_dbg_missing_data = 0;
static int g_dbg_printed_missing_data = 0;

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

static int find_or_add_joint_slot(ecs_entity_t joint)
{
	for (int i = 0; i < g_solver_cache.joint_count; i ++) {
		if (g_solver_cache.joints[i].joint == joint) {
			return i;
		}
	}

	if (g_solver_cache.joint_count >= MAX_SOLVER_JOINTS) {
		return -1;
	}

	const int slot = g_solver_cache.joint_count ++;
	g_solver_cache.joints[slot].joint = joint;
	g_solver_cache.joints[slot].residual_sq = 0.0;
	g_solver_cache.joints[slot].lambda_sq = 0.0;
	return slot;
}

static ecs_entity_t resolve_instance_pivot(
	const ecs_world_t *ecs,
	ecs_entity_t joint,
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

	const ecs_entity_t joints_scope = ecs_get_target(ecs, joint, EcsChildOf, 0);
	const ecs_entity_t simulation_scope = ecs_get_target(ecs, joints_scope, EcsChildOf, 0);
	if (simulation_scope == 0) {
		return 0;
	}

	const ecs_entity_t instance_body = ecs_lookup_child(ecs, simulation_scope, body_name);
	if (instance_body == 0) {
		return 0;
	}

	return ecs_lookup_child(ecs, instance_body, pivot_name);
}

static void resolve_joint_mates(
	const ecs_world_t *ecs,
	ecs_entity_t joint,
	ecs_entity_t *pivot_a,
	ecs_entity_t *pivot_b)
{
	*pivot_a = ecs_get_target(ecs, joint, ecs_id(Mate), 0);
	*pivot_b = ecs_get_target(ecs, joint, ecs_id(Mate), 1);

	const ecs_entity_t base_joint = ecs_get_target(ecs, joint, EcsIsA, 0);
	if (base_joint != 0) {
		const ecs_entity_t base_pivot_a = ecs_get_target(ecs, base_joint, ecs_id(Mate), 0);
		const ecs_entity_t base_pivot_b = ecs_get_target(ecs, base_joint, ecs_id(Mate), 1);

		if (*pivot_a == 0) {
			*pivot_a = base_pivot_a;
		}
		if (*pivot_b == 0) {
			*pivot_b = base_pivot_b;
		}
	}

	if (*pivot_a != 0) {
		ecs_entity_t remapped_a = resolve_instance_pivot(ecs, joint, *pivot_a);
		if (remapped_a != 0) {
			*pivot_a = remapped_a;
		}
	}
	if (*pivot_b != 0) {
		ecs_entity_t remapped_b = resolve_instance_pivot(ecs, joint, *pivot_b);
		if (remapped_b != 0) {
			*pivot_b = remapped_b;
		}
	}
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
	if (g_assembly_frame_marker != g_frame_counter) {
		g_assembly_frame_marker = g_frame_counter;
		g_solver_cache.row_count = 0;
		g_solver_cache.joint_count = 0;
		g_solver_cache.iter_count = 0;
		g_dbg_joint_total = 0;
		g_dbg_missing_mate = 0;
		g_dbg_missing_body = 0;
		g_dbg_missing_revolute = 0;
		g_dbg_missing_data = 0;
		g_dbg_printed_missing_data = 0;
	}

	const double dt = (it->delta_time > 0.0) ? (double)it->delta_time : (1.0 / 60.0);
	const double beta = 0.25;

	for (int i = 0; i < it->count; i ++) {
		g_dbg_joint_total ++;

		if (g_solver_cache.row_count + 2 > MAX_CONSTRAINT_ROWS) {
			break;
		}

		const ecs_entity_t joint = it->entities[i];
		ecs_entity_t pivot_a = 0;
		ecs_entity_t pivot_b = 0;
		resolve_joint_mates(it->world, joint, &pivot_a, &pivot_b);
		if (pivot_a == 0 || pivot_b == 0) {
			g_dbg_missing_mate ++;
			continue;
		}

		const ecs_entity_t body_a = ecs_get_target(it->world, pivot_a, EcsChildOf, 0);
		const ecs_entity_t body_b = ecs_get_target(it->world, pivot_b, EcsChildOf, 0);
		if (body_a == 0 || body_b == 0) {
			g_dbg_missing_body ++;
			continue;
		}

		const Pose *pose_a = ecs_get(it->world, body_a, Pose);
		const Pose *pose_b = ecs_get(it->world, body_b, Pose);
		const Mass *mass_a = ecs_get(it->world, body_a, Mass);
		const Mass *mass_b = ecs_get(it->world, body_b, Mass);
		const Inertia *inertia_a = ecs_get(it->world, body_a, Inertia);
		const Inertia *inertia_b = ecs_get(it->world, body_b, Inertia);
		const LocalOffset *local_a = ecs_get(it->world, pivot_a, LocalOffset);
		const LocalOffset *local_b = ecs_get(it->world, pivot_b, LocalOffset);

		if (pose_a == NULL || pose_b == NULL || mass_a == NULL || mass_b == NULL ||
			inertia_a == NULL || inertia_b == NULL || local_a == NULL || local_b == NULL) {
			g_dbg_missing_data ++;
			if (!g_dbg_printed_missing_data) {
				g_dbg_printed_missing_data = 1;
				printf("assembly missing data joint=%s pose_a=%d pose_b=%d mass_a=%d mass_b=%d inertia_a=%d inertia_b=%d local_a=%d local_b=%d\n",
					ecs_get_name(it->world, joint),
					pose_a != NULL,
					pose_b != NULL,
					mass_a != NULL,
					mass_b != NULL,
					inertia_a != NULL,
					inertia_b != NULL,
					local_a != NULL,
					local_b != NULL);
			}
			continue;
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

		const Revolute *joint_revolute = ecs_get(it->world, joint, Revolute);
		if (joint_revolute == NULL) {
			g_dbg_missing_revolute ++;
			continue;
		}

		double axis_x = joint_revolute->x;
		double axis_y = joint_revolute->y;
		const double axis_len = sqrt(axis_x * axis_x + axis_y * axis_y);
		if (axis_len < 1e-9) {
			axis_x = 1.0;
			axis_y = 0.0;
		} else {
			axis_x /= axis_len;
			axis_y /= axis_len;
		}

		const double tangent_x = -axis_y;
		const double tangent_y = axis_x;
		const Compliance *joint_compliance = ecs_get(it->world, joint, Compliance);
		const double compliance_value = (joint_compliance != NULL) ? joint_compliance->value : 0.0;
		const double alpha = (compliance_value > 0.0) ? (compliance_value / (dt * dt)) : 0.0;
		const Impulse *joint_impulse = ecs_get(it->world, joint, Impulse);
		const double warm_lambda = (joint_impulse != NULL) ? (0.5 * joint_impulse->value) : 0.0;
		const int joint_slot = find_or_add_joint_slot(joint);
		if (joint_slot < 0) {
			continue;
		}

		const double axes[2][2] = {
			{axis_x, axis_y},
			{tangent_x, tangent_y}
		};

		for (int row_axis = 0; row_axis < 2; row_axis ++) {
			ConstraintRow *row = &g_solver_cache.rows[g_solver_cache.row_count ++];
			const double nx = axes[row_axis][0];
			const double ny = axes[row_axis][1];

			row->joint = joint;
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
			row->lambda = warm_lambda;
			row->joint_slot = joint_slot;

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
	}
}

void SolveConstraintRows(ecs_iter_t *it)
{
	const SolverConfig *cfg = ecs_get(it->world, g_solver_config_entity, SolverConfig);
	const int configured_iters = (cfg != NULL && cfg->iterations > 0) ? cfg->iterations : 10;
	const int solver_iters = configured_iters > MAX_SOLVER_ITERS ? MAX_SOLVER_ITERS : configured_iters;

	if (g_solver_cache.row_count == 0) {
		printf("[frame %llu] global convergence no_rows joints=%d missing_mate=%d missing_body=%d missing_revolute=%d missing_data=%d\n",
			(unsigned long long)g_frame_counter,
			g_dbg_joint_total,
			g_dbg_missing_mate,
			g_dbg_missing_body,
			g_dbg_missing_revolute,
			g_dbg_missing_data);
		g_frame_counter ++;
		return;
	}

	for (int i = 0; i < g_solver_cache.joint_count; i ++) {
		g_solver_cache.joints[i].residual_sq = 0.0;
		g_solver_cache.joints[i].lambda_sq = 0.0;
	}

	for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
		apply_impulse_delta(it->world, &g_solver_cache.rows[row_index], g_solver_cache.rows[row_index].lambda);
	}

	for (int iter = 0; iter < solver_iters; iter ++) {
		double global_sq = 0.0;

		for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
			ConstraintRow *row = &g_solver_cache.rows[row_index];
			const double residual = compute_row_residual(it->world, row);
			const double delta_lambda = -residual / row->effective_mass;
			row->lambda += delta_lambda;
			apply_impulse_delta(it->world, row, delta_lambda);

			const double post_residual = compute_row_residual(it->world, row);
			global_sq += post_residual * post_residual;
			if (iter == solver_iters - 1) {
				g_solver_cache.joints[row->joint_slot].residual_sq += post_residual * post_residual;
			}
		}

		g_solver_cache.iter_residual[iter] = sqrt(global_sq);
	}

	g_solver_cache.iter_count = solver_iters;

	for (int row_index = 0; row_index < g_solver_cache.row_count; row_index ++) {
		ConstraintRow *row = &g_solver_cache.rows[row_index];
		g_solver_cache.joints[row->joint_slot].lambda_sq += row->lambda * row->lambda;
	}

	for (int joint_index = 0; joint_index < g_solver_cache.joint_count; joint_index ++) {
		const ecs_entity_t joint = g_solver_cache.joints[joint_index].joint;
		Impulse *joint_impulse = ecs_get_mut(it->world, joint, Impulse);
		if (joint_impulse != NULL) {
			joint_impulse->value = sqrt(g_solver_cache.joints[joint_index].lambda_sq);
			ecs_modified(it->world, joint, Impulse);
		}
	}

	printf("[frame %llu] global convergence", (unsigned long long)g_frame_counter);
	for (int iter = 0; iter < g_solver_cache.iter_count; iter ++) {
		printf(" i%d=%.3e", iter, g_solver_cache.iter_residual[iter]);
	}
	printf("\n");

	for (int joint_index = 0; joint_index < g_solver_cache.joint_count; joint_index ++) {
		const char *joint_name = ecs_get_name(it->world, g_solver_cache.joints[joint_index].joint);
		printf("  joint=%s residual_norm=%.3e\n",
			joint_name != NULL ? joint_name : "<unnamed>",
			sqrt(g_solver_cache.joints[joint_index].residual_sq));
	}

	g_frame_counter ++;
}

void IntegrateBodies(ecs_iter_t *it)
{
	const SolverConfig *cfg = ecs_get(it->world, g_solver_config_entity, SolverConfig);
	const double dt = (cfg != NULL && cfg->dt > 0.0) ? cfg->dt : ((it->delta_time > 0.0) ? (double)it->delta_time : (1.0 / 60.0));

	for (int i = 0; i < it->count; i ++) {
		const Velocity *velocity = ecs_get(it->world, it->entities[i], Velocity);
		Pose *pose = ecs_get_mut(it->world, it->entities[i], Pose);
		if (velocity == NULL || pose == NULL) {
			continue;
		}

		pose->x += velocity->x * dt;
		pose->y += velocity->y * dt;
		pose->angle += velocity->angular * dt;
		ecs_modified(it->world, it->entities[i], Pose);
	}
}

int main(int argc, char *argv[])
{
	ecs_world_t *ecs = ecs_init_w_args(argc, argv);

	ECS_IMPORT(ecs, SimTypes);

	ECS_SYSTEM(ecs, AssembleRevoluteRows, EcsPreUpdate, Impulse);
	ECS_SYSTEM(ecs, SolveConstraintRows, EcsOnUpdate, SolverConfig);
	ECS_SYSTEM(ecs, IntegrateBodies, EcsPostUpdate, Velocity);

	g_solver_config_entity = ecs_entity(ecs, {.name = "SolverConfigEntity"});
	ecs_set(ecs, g_solver_config_entity, SolverConfig, {
		.dt = 1.0 / 60.0,
		.baumgarte = 0.25,
		.iterations = 12,
	});


	if (ecs_script_run_file(ecs, "assets/entities.flecs")) {
		return 1;
	}

	printf("startup counts: Impulse=%d Revolute=%d SolverConfig=%d\n",
		ecs_count_id(ecs, ecs_id(Impulse)),
		ecs_count_id(ecs, ecs_id(Revolute)),
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