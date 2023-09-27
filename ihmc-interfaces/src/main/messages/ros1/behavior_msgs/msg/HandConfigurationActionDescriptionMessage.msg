
# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# The grip type
uint32 grip

# Whether the next action can be executed at the same time of this one
bool execute_with_next_action


