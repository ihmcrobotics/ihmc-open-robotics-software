# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

byte COUNTER_TYPE = 0
byte LLM_TYPE = 1

# The type of condtion as defined above
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
