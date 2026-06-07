#include <stdio.h>
#include <flecs.h>

ECS_COMPONENT_DECLARE(Pose);
ECS_COMPONENT_DECLARE(Velocity);
ECS_COMPONENT_DECLARE(Mass);
ECS_COMPONENT_DECLARE(Inertia);
ECS_COMPONENT_DECLARE(LocalOffset);
ECS_COMPONENT_DECLARE(Axis);
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
 * @brief Preferred or constrained axis for directional constraints.
 *
 * Examples include hinge axis conventions, slider directions, or motor axes.
 * Row assembly normalizes/uses this vector to form directional Jacobians.
 */
typedef struct {
    double x, y;
} Axis;

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

static const char* name_of(ecs_world_t *world, ecs_entity_t entity) {
    const char *name = ecs_get_name(world, entity);
    return name ? name : "<unnamed>";
}

static void print_pivot_ref(ecs_world_t *world, ecs_entity_t pivot) {
    const LocalOffset *offset = ecs_get(world, pivot, LocalOffset);

    printf("%s[%.2f, %.2f]",
        name_of(world, pivot),
        offset ? offset->x : 0.0,
        offset ? offset->y : 0.0);
}

static void print_body(ecs_world_t *world, ecs_entity_t entity) {
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

static void print_joint(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b) {
    const Axis *axis = ecs_get(world, entity, Axis);
    const Compliance *compliance = ecs_get(world, entity, Compliance);
    const Impulse *impulse = ecs_get(world, entity, Impulse);

    printf("Joint %-11s ", name_of(world, entity));
    print_pivot_ref(world, pivot_a);
    printf(" <-> ");
    print_pivot_ref(world, pivot_b);
    printf(" axis=(%.2f, %.2f) compliance=%.4f impulse=%.2f\n",
        axis ? axis->x : 0.0,
        axis ? axis->y : 0.0,
        compliance ? compliance->value : 0.0,
        impulse ? impulse->value : 0.0);
}

static void print_motor(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b) {
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

static void print_limit(ecs_world_t *world, ecs_entity_t entity, ecs_entity_t pivot_a, ecs_entity_t pivot_b) {
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

int main(int argc, char *argv[]) {
    ecs_world_t *ecs = ecs_init_w_args(argc, argv);

    ECS_COMPONENT_DEFINE(ecs, Pose);
    ECS_COMPONENT_DEFINE(ecs, Velocity);
    ECS_COMPONENT_DEFINE(ecs, Mass);
    ECS_COMPONENT_DEFINE(ecs, Inertia);
    ECS_COMPONENT_DEFINE(ecs, LocalOffset);
    ECS_COMPONENT_DEFINE(ecs, Axis);
    ECS_COMPONENT_DEFINE(ecs, Range);
    ECS_COMPONENT_DEFINE(ecs, Target);
    ECS_COMPONENT_DEFINE(ecs, Compliance);
    ECS_COMPONENT_DEFINE(ecs, Impulse);

    ecs_entity_t ground = ecs_entity(ecs, {.name = "ground"});
    ecs_set(ecs, ground, Pose, {0, 0, 0});
    ecs_set(ecs, ground, Velocity, {0, 0, 0});
    ecs_set(ecs, ground, Mass, {0});
    ecs_set(ecs, ground, Inertia, {0});

    ecs_entity_t lever = ecs_entity(ecs, {.name = "lever"});
    ecs_set(ecs, lever, Pose, {1.0, 0.0, 0.0});
    ecs_set(ecs, lever, Velocity, {0, 0, 0});
    ecs_set(ecs, lever, Mass, {2.5});
    ecs_set(ecs, lever, Inertia, {0.8});

    ecs_entity_t ground_pin = ecs_entity(ecs, {.name = "ground_pin"});
    ecs_add_pair(ecs, ground_pin, EcsChildOf, ground);
    ecs_set(ecs, ground_pin, LocalOffset, {0.0, 0.0});

    ecs_entity_t lever_root = ecs_entity(ecs, {.name = "lever_root"});
    ecs_add_pair(ecs, lever_root, EcsChildOf, lever);
    ecs_set(ecs, lever_root, LocalOffset, {-1.0, 0.0});

    ecs_entity_t lever_tip = ecs_entity(ecs, {.name = "lever_tip"});
    ecs_add_pair(ecs, lever_tip, EcsChildOf, lever);
    ecs_set(ecs, lever_tip, LocalOffset, {1.0, 0.0});

    ecs_entity_t pivot = ecs_entity(ecs, {.name = "pivot"});
    ecs_add_id(ecs, pivot, EcsSymmetric);
    ecs_add_pair(ecs, ground_pin, pivot, lever_root);
    ecs_set(ecs, pivot, Axis, {0.0, 1.0});
    ecs_set(ecs, pivot, Compliance, {0.0001});
    ecs_set(ecs, pivot, Impulse, {0.0});

    ecs_entity_t drive = ecs_entity(ecs, {.name = "drive"});
    ecs_add_id(ecs, drive, EcsSymmetric);
    ecs_add_pair(ecs, ground_pin, drive, lever_tip);
    ecs_set(ecs, drive, Target, {1.0});
    ecs_set(ecs, drive, Impulse, {0.0});

    ecs_entity_t stop = ecs_entity(ecs, {.name = "stop"});
    ecs_add_id(ecs, stop, EcsSymmetric);
    ecs_add_pair(ecs, ground_pin, stop, lever_tip);
    ecs_set(ecs, stop, Range, {-0.5, 0.5});
    ecs_set(ecs, stop, Impulse, {0.0});

    printf("Flecs graph sketch\n");
    printf("- Bodies are persistent nodes\n");
    printf("- Pivot points are entities owned by bodies with local offsets\n");
    printf("- Joints, constraints, motors, and limits connect pivot entities\n");
    printf("- The solver would later compile these entities into transient constraint rows\n\n");

    print_body(ecs, ground);
    print_body(ecs, lever);
    print_joint(ecs, pivot, ground_pin, lever_root);
    print_motor(ecs, drive, ground_pin, lever_tip);
    print_limit(ecs, stop, ground_pin, lever_tip);

    return ecs_fini(ecs);
}