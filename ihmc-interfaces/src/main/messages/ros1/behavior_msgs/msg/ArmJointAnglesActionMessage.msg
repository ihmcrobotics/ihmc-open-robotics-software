
# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Joint angles
float64[] joint_angles

# Duration of the trajectory
float64 trajectory_duration


