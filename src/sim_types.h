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

typedef struct {
	double x, y;
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
