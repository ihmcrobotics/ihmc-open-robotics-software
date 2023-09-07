
# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Name of the frame the this action is expressed in
string[] parent_frame

# Transform that expresses the chest pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# The trajectory duration
float64 trajectory_duration


