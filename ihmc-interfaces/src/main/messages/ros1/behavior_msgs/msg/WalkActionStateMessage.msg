
int8 TRIGGERED=0

int8 FOOTSTEP_PLANNING=1

int8 PLANNING_FAILED=2

int8 PLANNING_SUCCEEDED=3

int8 PLAN_COMMANDED=4

int8 PLAN_EXECUTION_COMPLETE=5

# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/WalkActionDefinitionMessage definition

int8 execution_state

behavior_msgs/FootstepPlanActionStateBasicsMessage footstep_plan_state_basics


