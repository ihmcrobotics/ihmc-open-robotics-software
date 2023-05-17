# A detectable perception scene node
# The topic name identifies the node.

# The name of the scene node
string name

# Whether or not the node is currently detected
bool currently_detected

# Transform of the node's frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world

# Keeps track of when the operator has overridden the pose of this node
bool is_pose_overridden_by_operator