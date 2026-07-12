# Value of the node to goto ID when we have a name but still need to find the ID
int64 INVALID = -1
# Value of the node to goto ID when the node is simply set to goto the next node (i.e. no-op)
int64 GOTO_NEXT = -2

# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

# The ID of the node to goto or one of the invalid or next constants
int64 node_to_goto_id
