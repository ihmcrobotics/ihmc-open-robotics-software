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

# The positional vector from frame B to frame A in world frame
geometry_msgs/Vector3 b_to_a

# Whether frame A is present
bool frame_a_is_present

# Whether frame B is present
bool frame_b_is_present

# SHAPE CONTAINS TYPE

bool frame_is_contained

uint32 number_of_points_contained

float32 average_hue

float32 average_saturation

float32 average_value
