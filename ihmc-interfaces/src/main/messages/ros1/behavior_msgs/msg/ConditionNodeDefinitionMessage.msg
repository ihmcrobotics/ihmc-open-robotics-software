# COUNTER TYPE
# LLM TYPE
# PROXIMITY TYPE

int8 COUNTER_TYPE=0

int8 LLM_TYPE=1

int8 PROXIMITY_TYPE=2

int8 XYZ_TYPE=0

int8 XY_TYPE=1

int8 Z_TYPE=2

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
int8 distance_type

# Name of the object frame
string object_frame_name

# Name of the frame the distance is expressed in
string reference_frame_name

# The maximum distance between the object and the reference frame
float64 distance_to_object

# The maximum time that is spent in evaluating the condition
float64 evaluation_time


