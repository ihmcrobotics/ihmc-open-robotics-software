# An perception scene node for an ArUco marker
# The topic name identifies the node.

# The detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# ArUco marker ID
uint16 marker_id

# ArUco marker size
float32 marker_size

# Break frequency filter value for nodes that are alpha filtered
float32 break_frequency


