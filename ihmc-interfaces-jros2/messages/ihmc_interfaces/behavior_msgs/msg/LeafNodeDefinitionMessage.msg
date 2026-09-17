# Value of execute after node ID when we have a name but still need to find the ID
int64 INVALID = -1
# Value of execute after node ID when this leaf is set to execute after the previous node
int64 EXECUTE_AFTER_PREVIOUS = -2
# Value of execute after node ID when this leaf is set to execute after the beginning of the sequence
int64 EXECUTE_AFTER_BEGINNING = -3

# Parent definition fields
behavior_msgs/BehaviorTreeNodeDefinitionMessage definition

# The ID of the node to execute after or one of the invalid, previous, or beginning constants
int64 execute_after_node_id
