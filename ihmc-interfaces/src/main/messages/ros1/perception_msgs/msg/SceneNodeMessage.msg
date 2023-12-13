# The base scene node information
# The topic name identifies the node.

# The ID of the node
uint32 id

# The name of the scene node
string name

# Transform of the node's frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world

# Number of child nodes; used for serialization
uint16 number_of_children


