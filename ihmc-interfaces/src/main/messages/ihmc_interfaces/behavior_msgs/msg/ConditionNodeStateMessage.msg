# Parent state fields
behavior_msgs/LeafNodeStateMessage state

# Definition
behavior_msgs/ConditionNodeDefinitionMessage definition

# COUNTER TYPE

# The current count of the counter
uint32 count

# LLM TYPE

# A request to reset the LLM's context now
bool request_reset_context

# PROXIMITY TYPE

# The current distance of the object from the reference frame
float64 current_distance
