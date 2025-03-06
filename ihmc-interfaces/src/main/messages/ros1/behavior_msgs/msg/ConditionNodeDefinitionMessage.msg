# COUNTER TYPE
# LLM TYPE

int8 COUNTER_TYPE=0

int8 LLM_TYPE=1

# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

# The type of condtion as defined above
int8 type

# The number of times to fail before passing
uint32 count_to

# Whether to reset the context on each run
bool reset_context_each_run

# Whether to generate behavior state information and add it to the prompt
bool inject_behavior_state

# Whether to generate environment state information and add it to the prompt
bool inject_environment_state

# The prompt given once at the very beginning
string system

# The prompt defining how to make the boolean decision
string prompt


