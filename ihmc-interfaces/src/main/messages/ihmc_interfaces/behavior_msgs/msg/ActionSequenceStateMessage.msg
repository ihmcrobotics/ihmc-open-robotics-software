# Parent state fields
behavior_msgs/BehaviorTreeNodeStateMessage state

# Definition
behavior_msgs/ActionSequenceDefinitionMessage definition

# If the sequence is currently set to proceed automatically
bool automatic_execution

# The index of the action that is set to execute next
uint16 execution_next_index

# Next action rejection tooltip
string next_action_rejection_tooltip

# Request manual execution of the next action
bool manual_execution_requested

# The user requested kinematic simulation preview time
float64 preview_requested_time
