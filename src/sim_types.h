#pragma once

#include <flecs.h>

typedef struct {
	double x, y, angle;
} Pose;

typedef struct {
	double x, y, angular;
} Velocity;

typedef struct {
	double value;
} Mass;

typedef struct {
	double value;
} Inertia;

typedef struct {
	double x, y;
} LocalOffset;

typedef struct {
	int dummy;
} Mate;

/**
 * @brief Tag data for a 2D revolute joint relation.
 *
 * The pin-hole anchors are computed from each mated pivot's LocalOffset and
 * body Pose. The 2D rotation axis is the implicit out-of-plane Z axis.
 */
typedef struct {
	/** Placeholder field so the component has a concrete struct layout. */
	int dummy;
} Revolute;

typedef struct {
	double min;
	double max;
} Range;

typedef struct {
	double value;
} Target;

typedef struct {
	double value;
} Compliance;

typedef struct {
	double value;
} Impulse;

typedef struct {
	double dt;
	double baumgarte;
	int iterations;
} SolverConfig;

typedef struct {
	int dummy;
} SimTypes;

extern ECS_COMPONENT_DECLARE(Pose);
extern ECS_COMPONENT_DECLARE(Velocity);
extern ECS_COMPONENT_DECLARE(Mass);
extern ECS_COMPONENT_DECLARE(Inertia);
extern ECS_COMPONENT_DECLARE(LocalOffset);
extern ECS_COMPONENT_DECLARE(Mate);
extern ECS_COMPONENT_DECLARE(Revolute);
extern ECS_COMPONENT_DECLARE(Range);
extern ECS_COMPONENT_DECLARE(Target);
extern ECS_COMPONENT_DECLARE(Compliance);
extern ECS_COMPONENT_DECLARE(Impulse);
extern ECS_COMPONENT_DECLARE(SolverConfig);

void SimTypesImport(ecs_world_t *ecs);
