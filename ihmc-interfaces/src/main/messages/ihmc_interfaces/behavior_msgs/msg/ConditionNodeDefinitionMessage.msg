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

# The prompt defining how to make the boolean decision
string<=10000 prompt
