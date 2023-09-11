
# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Name of the frame the this action is expressed in
string[] parent_frame

# Transform that expresses the pelvis pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# Duration of the trajectory
float64 trajectory_duration

# Whether the next action can be executed at the same time of this one
bool execute_with_next_action


