# Parent state fields
behavior_msgs/BehaviorTreeNodeStateMessage state

# The action's unique ID
uint32 id

# If the action is next for execution
bool is_next_for_execution

# If the node is able to execution
bool can_execute

# If the node is currently executing
bool is_executing

# If the node had a failure during it's last execution
bool failed
