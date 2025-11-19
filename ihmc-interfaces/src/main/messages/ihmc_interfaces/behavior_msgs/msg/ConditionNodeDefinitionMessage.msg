# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

byte COUNTER_TYPE = 0
byte LLM_TYPE = 1
byte PROXIMITY_TYPE = 2
byte ALWAYS_FAIL = 3
byte ALWAYS_SUCCEED = 4

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

byte PROXIMITY_XYZ = 0
byte PROXIMITY_XY = 1
byte PROXIMITY_Z = 2

# The type of distance condition as defined above
byte proximity_distance_type

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