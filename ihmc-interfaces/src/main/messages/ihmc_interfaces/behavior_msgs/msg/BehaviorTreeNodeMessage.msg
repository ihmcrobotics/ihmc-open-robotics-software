# DEPRECATED - See BehaviorTreeNodeDefinitionMessage for the new version
# We will remove this when the implementation has been migrated to the
# new structure.

# The number of children
# This is used for a stack data structure to re-assemble the tree.
uint32 number_of_children

# Last tick instant
ihmc_common_msgs/InstantMessage last_tick_instant

# Name of the node
string node_name

# The type of the node, as a string
string node_type

# Previous node status
int8 previous_status

# Whether this node has been clocked
# This field is only for control flow nodes.
bool has_been_clocked