# The node's unique ID
uint32 id

# A node is active if it lies on the path of the current tree tick.
bool is_active

# Recent log messages
behavior_msgs/BehaviorTreeLogMessage[<=50] recent_log_messages
