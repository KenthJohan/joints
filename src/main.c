#include <stdio.h>
#include <flecs.h>

ECS_COMPONENT_DECLARE(Pose);
ECS_COMPONENT_DECLARE(Velocity);
ECS_COMPONENT_DECLARE(Mass);
ECS_COMPONENT_DECLARE(Inertia);
ECS_COMPONENT_DECLARE(LocalOffset);
ECS_COMPONENT_DECLARE(Mate);
ECS_COMPONENT_DECLARE(Revolute);
ECS_COMPONENT_DECLARE(Range);
ECS_COMPONENT_DECLARE(Target);
ECS_COMPONENT_DECLARE(Compliance);
ECS_COMPONENT_DECLARE(Impulse);

/**
 * @brief World-space pose for a rigid body node in the mechanical graph.
 *
 * The solver treats this as persistent state. Constraint row generation reads
 * pose values to compute Jacobians and positional error terms each step.
 */
typedef struct {
	double x, y, angle;
} Pose;

/**
 * @brief Linear and angular velocity for a rigid body node.
 *
 * During solving, impulses from constraint rows are accumulated into this
 * component and later integrated back into Pose.
 */
typedef struct {
	double x, y, angular;
} Velocity;

/**
 * @brief Scalar mass for translational dynamics.
 *
 * A zero value represents an immovable/static body. Constraint rows use inverse
 * mass to compute effective mass and impulse response.
 */
typedef struct {
	double value;
} Mass;

/**
 * @brief Scalar rotational inertia for angular dynamics.
 *
 * A zero value represents infinite inertia (no angular response). Row assembly
 * uses inverse inertia for angular Jacobian terms.
 */
typedef struct {
	double value;
} Inertia;

/**
 * @brief Local offset on a body for authoring pivot point markers.
 *
 * A pivot entity stores this offset in its owning body's local frame. Joint
 * row assembly can resolve world anchors by combining owner Pose and this data.
 */
typedef struct {
	double x, y;
} LocalOffset;

/**
 * @brief Relationship tag for revolute (pin) mates between pivot entities.
 *
 * The dummy field gives the type a concrete size so it can be registered
 * as a component. No meaningful data is stored here.
 */
typedef struct {
	int dummy;
} Mate;

/**
 * @brief Preferred or constrained axis for revolute constraints.
 *
 * Examples include hinge axis conventions, slider directions, or motor axes.
 * Row assembly normalizes/uses this vector to form directional Jacobians.
 */
typedef struct {
	double x, y;
} Revolute;

/**
 * @brief Min/max bounds for limit constraints.
 *
 * Limit rows become inequality constraints where impulses are clamped when the
 * measured value reaches @c min or @c max.
 */
typedef struct {
	double min;
	double max;
} Range;

/**
 * @brief Target setpoint for actuated constraints (for example motors/servos).
 *
 * The solver converts target error into a row bias term, producing impulses
 * that drive the connected bodies toward the commanded value.
 */
typedef struct {
	double value;
} Target;

/**
 * @brief Softness/compliance parameter for a constraint.
 *
 * Higher values make the constraint softer. Row assembly maps this to a
 * regularization term so hard constraints and compliant constraints share one
 * solver path.
 */
typedef struct {
	double value;
} Compliance;

/**
 * @brief Cached Lagrange multiplier (impulse) per constraint entity.
 *
 * Persisted between frames for warm starting. At solve time, this initial
 * guess is applied first, then updated by iterative row solves.
 */
typedef struct {
	double value;
} Impulse;

static const char *name_of(ecs_world_t *world, ecs_entity_t entity)
{
	const char *name = ecs_get_name(world, entity);
	return name ? name : "<unnamed>";
}

static void print_pivot_ref(ecs_world_t *world, ecs_entity_t pivot)
{
	const LocalOffset *offset = ecs_get(world, pivot, LocalOffset);

	printf("%s[%.2f, %.2f]",
	name_of(world, pivot),
	offset ? offset->x : 0.0,
	offset ? offset->y : 0.0);
}

static void print_body(ecs_world_t *world, ecs_entity_t entity)
{
	const Pose *pose = ecs_get(world, entity, Pose);
	const Velocity *velocity = ecs_get(world, entity, Velocity);
	const Mass *mass = ecs_get(world, entity, Mass);
	const Inertia *inertia = ecs_get(world, entity, Inertia);

	printf("Body %-12s pose=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) mass=%.2f inertia=%.2f\n",
	name_of(world, entity),
	pose ? pose->x : 0.0,
	pose ? pose->y : 0.0,
	pose ? pose->angle : 0.0,
	velocity ? velocity->x : 0.0,
	velocity ? velocity->y : 0.0,
	velocity ? velocity->angular : 0.0,
	mass ? mass->value : 0.0,
	inertia ? inertia->value : 0.0);
}

static void print_joint(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b)
{
	const Revolute *revolute = ecs_get(world, entity, Revolute);
	const Compliance *compliance = ecs_get(world, entity, Compliance);
	const Impulse *impulse = ecs_get(world, entity, Impulse);

	printf("Joint %-11s ", name_of(world, entity));
	print_pivot_ref(world, pivot_a);
	printf(" <-> ");
	print_pivot_ref(world, pivot_b);
	printf(" revolute=(%.2f, %.2f) compliance=%.4f impulse=%.2f\n",
	revolute ? revolute->x : 0.0,
	revolute ? revolute->y : 0.0,
	compliance ? compliance->value : 0.0,
	impulse ? impulse->value : 0.0);
}

static void print_motor(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b)
{
	const Target *target = ecs_get(world, entity, Target);
	const Impulse *impulse = ecs_get(world, entity, Impulse);

	printf("Motor %-11s ", name_of(world, entity));
	print_pivot_ref(world, pivot_a);
	printf(" -> ");
	print_pivot_ref(world, pivot_b);
	printf(" target=%.2f impulse=%.2f\n",
	target ? target->value : 0.0,
	impulse ? impulse->value : 0.0);
}

static void print_limit(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b)
{
	const Range *range = ecs_get(world, entity, Range);
	const Impulse *impulse = ecs_get(world, entity, Impulse);

	printf("Limit %-11s ", name_of(world, entity));
	print_pivot_ref(world, pivot_a);
	printf(" <-> ");
	print_pivot_ref(world, pivot_b);
	printf(" range=[%.2f, %.2f] impulse=%.2f\n",
	range ? range->min : 0.0,
	range ? range->max : 0.0,
	impulse ? impulse->value : 0.0);
}

int main(int argc, char *argv[])
{
	ecs_world_t *ecs = ecs_init_w_args(argc, argv);

	ECS_COMPONENT_DEFINE(ecs, Pose);
	ECS_COMPONENT_DEFINE(ecs, Velocity);
	ECS_COMPONENT_DEFINE(ecs, Mass);
	ECS_COMPONENT_DEFINE(ecs, Inertia);
	ECS_COMPONENT_DEFINE(ecs, LocalOffset);
	ECS_COMPONENT_DEFINE(ecs, Mate);
	ECS_COMPONENT_DEFINE(ecs, Range);
	ECS_COMPONENT_DEFINE(ecs, Target);
	ECS_COMPONENT_DEFINE(ecs, Compliance);
	ECS_COMPONENT_DEFINE(ecs, Impulse);

	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Pose),
	.members = {{"x", ecs_id(ecs_f64_t)}, {"y", ecs_id(ecs_f64_t)}, {"angle", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Velocity),
	.members = {{"x", ecs_id(ecs_f64_t)}, {"y", ecs_id(ecs_f64_t)}, {"angular", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Mass),
	.members = {{"value", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Inertia),
	.members = {{"value", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(LocalOffset),
	.members = {{"x", ecs_id(ecs_f64_t)}, {"y", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Mate),
	.members = {{"dummy", ecs_id(ecs_i32_t)}},
	});
	ECS_COMPONENT_DEFINE(ecs, Revolute);
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Revolute),
	.members = {{"x", ecs_id(ecs_f64_t)}, {"y", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Range),
	.members = {{"min", ecs_id(ecs_f64_t)}, {"max", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Target),
	.members = {{"value", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Compliance),
	.members = {{"value", ecs_id(ecs_f64_t)}},
	});
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Impulse),
	.members = {{"value", ecs_id(ecs_f64_t)}},
	});

	if (ecs_script_run_file(ecs, "assets/entities.flecs")) {
		return 1;
	}

	ecs_entity_t ground = ecs_lookup(ecs, "ground");
	ecs_entity_t lever = ecs_lookup(ecs, "lever");
	ecs_entity_t ground_pin = ecs_lookup(ecs, "ground.ground_pin");
	ecs_entity_t lever_root = ecs_lookup(ecs, "lever.lever_root");
	ecs_entity_t lever_tip = ecs_lookup(ecs, "lever.lever_tip");
	ecs_entity_t mate = ecs_lookup(ecs, "Mate");
	ecs_entity_t drive = ecs_lookup(ecs, "drive");
	ecs_entity_t stop = ecs_lookup(ecs, "stop");

	printf("Flecs graph sketch\n");
	printf("- Bodies are persistent nodes\n");
	printf("- Pivot points are entities owned by bodies with local offsets\n");
	printf("- Joints, constraints, motors, and limits connect pivot entities\n");
	printf("- The solver would later compile these entities into transient constraint rows\n\n");

	print_body(ecs, ground);
	print_body(ecs, lever);
	print_joint(ecs, mate, ground_pin, lever_root);
	print_motor(ecs, drive, ground_pin, lever_tip);
	print_limit(ecs, stop, ground_pin, lever_tip);

	ecs_set(ecs, EcsWorld, EcsRest, {.port = 0});
	printf("Remote: %s\n", "https://www.flecs.dev/explorer/?remote=true");

	while (1) {
		ecs_progress(ecs, 1.0f);
	}

	return ecs_fini(ecs);
}