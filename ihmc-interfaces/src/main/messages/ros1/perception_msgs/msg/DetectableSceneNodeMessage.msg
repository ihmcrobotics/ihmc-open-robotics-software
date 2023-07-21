# A detectable perception scene node
# The topic name identifies the node.

# The name of the scene node
string name

# Whether or not the node is currently detected
bool currently_detected

# Transform of the node's frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world

# If this node is trackable via an ArUco maker, this is the ArUco marker's
# transform to world frame. This is so we can reset overriden node
# poses back to ArUco relative ones.
controller_msgs/RigidBodyTransformMessage aruco_marker_transform_to_world

# Keeps track of when the operator has overridden the pose of this node
bool is_pose_overridden_by_operator


