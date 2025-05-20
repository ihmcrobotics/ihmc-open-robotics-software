# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

byte COUNTER_TYPE = 0
byte LLM_TYPE = 1
byte PROXIMITY_TYPE = 2

# The type of condition as defined above
byte type

# COUNTER TYPE

# The number of times to fail before passing
uint32 count_to

# LLM TYPE

# Whether to reset the context on each run
bool reset_context_each_run

# Whether to generate behavior state information and add it to the prompt
bool inject_behavior_state

# Whether to generate environment state information and add it to the prompt
bool inject_environment_state

# If a response match indicates success, else a match indicates failure
bool match_is_success

# The prompt given once at the very beginning
string<=10000 system

# The prompt defining how to make the boolean decision
string<=10000 prompt

# A regular expression, where if it matches, the execution failed
string<=10000 response_matcher

# PROXIMITY TYPE

byte XYZ_TYPE = 0
byte XY_TYPE = 1
byte Z_TYPE = 2

# The type of distance condition as defined above
byte distance_type

# Name of the object frame
string object_frame_name

# Name of the frame the distance is expressed in
string reference_frame_name

# The maximum distance between the object and the reference frame
float64 distance_to_object

# The maximum time that is spent in evaluating the condition
float64 evaluation_time
