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

# Nodes can be set to not track the detected pose
bool track_detected_pose

# Break frequency filter value for nodes that are alpha filtered
float32 break_frequency


