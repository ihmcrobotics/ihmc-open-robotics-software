# Parent state fields
behavior_msgs/BehaviorTreeNodeStateMessage node_state

# The action's unique ID
uint32 id

# The action's index in the sequence
int32 action_index

# If the action is next for execution
bool is_next_for_execution

# If the action is to be executed concurrently
bool is_to_be_executed_concurrently

# If the node is able to execution
bool can_execute

# If the node is currently executing
bool is_executing
