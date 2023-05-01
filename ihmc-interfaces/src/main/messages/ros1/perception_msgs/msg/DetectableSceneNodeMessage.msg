# A detectable perception scene node
# The topic name identifies the node.

# The name of the scene node
string name

# Whether or not the node is currently detected
bool currently_detected

# Transform of the node's frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world


