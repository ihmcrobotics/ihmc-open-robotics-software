# Whether the action hs to be executed next and is part of a group of concurrent actions
bool current_and_concurrent

# Name of the frame the this action is expressed in
string[<=1] parent_frame

# Transform that expresses the pelvis pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent