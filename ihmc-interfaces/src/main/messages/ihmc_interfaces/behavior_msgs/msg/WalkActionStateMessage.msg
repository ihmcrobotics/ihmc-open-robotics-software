# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/WalkActionDefinitionMessage definition

byte TRIGGERED = 0
byte FOOTSTEP_PLANNING = 1
byte PLANNING_FAILED = 2
byte PLANNING_SUCCEEDED = 3
byte PLAN_COMMANDED = 4
byte PLAN_EXECUTION_COMPLETE = 5

byte execution_state

behavior_msgs/FootstepPlanActionStateBasicsMessage footstep_plan_state_basics