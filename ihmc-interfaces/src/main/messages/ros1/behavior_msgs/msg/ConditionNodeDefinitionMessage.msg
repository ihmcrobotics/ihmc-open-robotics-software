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

# The prompt defining how to make the boolean decision
string prompt


