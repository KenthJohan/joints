#include "sim_types.h"

ECS_COMPONENT_DECLARE(Pose) = 0;
ECS_COMPONENT_DECLARE(Velocity) = 0;
ECS_COMPONENT_DECLARE(Mass) = 0;
ECS_COMPONENT_DECLARE(Inertia) = 0;
ECS_COMPONENT_DECLARE(LocalOffset) = 0;
ECS_COMPONENT_DECLARE(Mate) = 0;
ECS_COMPONENT_DECLARE(Revolute) = 0;
ECS_COMPONENT_DECLARE(Range) = 0;
ECS_COMPONENT_DECLARE(Target) = 0;
ECS_COMPONENT_DECLARE(Compliance) = 0;
ECS_COMPONENT_DECLARE(Impulse) = 0;
ECS_COMPONENT_DECLARE(SolverConfig) = 0;

void SimTypesImport(ecs_world_t *ecs)
{
	ECS_MODULE(ecs, SimTypes);
	ecs_set_scope(ecs, 0);

	ECS_COMPONENT_DEFINE(ecs, Pose);
	ECS_COMPONENT_DEFINE(ecs, Velocity);
	ECS_COMPONENT_DEFINE(ecs, Mass);
	ECS_COMPONENT_DEFINE(ecs, Inertia);
	ECS_COMPONENT_DEFINE(ecs, LocalOffset);
	ECS_COMPONENT_DEFINE(ecs, Mate);
	ECS_COMPONENT_DEFINE(ecs, Revolute);
	ECS_COMPONENT_DEFINE(ecs, Range);
	ECS_COMPONENT_DEFINE(ecs, Target);
	ECS_COMPONENT_DEFINE(ecs, Compliance);
	ECS_COMPONENT_DEFINE(ecs, Impulse);
	ECS_COMPONENT_DEFINE(ecs, SolverConfig);

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
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(Revolute),
	.members = {{"dummy", ecs_id(ecs_i32_t)}},
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
	ecs_struct_init(ecs,
	&(ecs_struct_desc_t){
	.entity = ecs_id(SolverConfig),
	.members = {
		{"dt", ecs_id(ecs_f64_t)},
		{"baumgarte", ecs_id(ecs_f64_t)},
		{"iterations", ecs_id(ecs_i32_t)}
	},
	});
}
