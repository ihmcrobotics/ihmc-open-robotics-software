
# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Name of the frame the this action is expressed in
string[] parent_frame

# Transform that expresses the transform to the parent frame
# that the footsteps are expressed in.
controller_msgs/RigidBodyTransformMessage transform_to_parent

# The footsteps, with a maximum of 50
behavior_msgs/FootstepActionDefinitionMessage[] footsteps

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration


