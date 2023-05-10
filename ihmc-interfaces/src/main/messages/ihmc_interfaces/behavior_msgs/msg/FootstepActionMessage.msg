# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string[<=1000] parent_frame

# Transform that expresses the footstep pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent