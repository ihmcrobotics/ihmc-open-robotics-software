# COUNTER TYPE
# LLM TYPE
# PROXIMITY TYPE

int8 COUNTER_TYPE=0

int8 LLM_TYPE=1

int8 PROXIMITY_TYPE=2

int8 PROXIMITY_XYZ=0

int8 PROXIMITY_XY=1

int8 PROXIMITY_Z=2

# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

# The type of condition as defined above
int8 type

# The number of times to fail before passing
uint32 count_to

# Whether to reset the context on each run
bool reset_context_each_run

# Whether to generate behavior state information and add it to the prompt
bool inject_behavior_state

# Whether to generate environment state information and add it to the prompt
bool inject_environment_state

# If a response match indicates success, else a match indicates failure
bool match_is_success

# The prompt given once at the very beginning
string system

# The prompt defining how to make the boolean decision
string prompt

# A regular expression, where if it matches, the execution failed
string response_matcher

# The type of distance condition as defined above
int8 proximity_distance_type

# Name of the object frame
string proximity_object_frame_name

# Name of the frame the distance is expressed in
string proximity_reference_frame_name

# The maximum distance between the object and the reference frame
float64 proximity_distance_to_object

# The maximum time that is spent in evaluating the condition
float64 proximity_evaluation_time

# Whether the failure of missing frame is handled internally by the condition node or not
bool manage_missing_frame_internally


