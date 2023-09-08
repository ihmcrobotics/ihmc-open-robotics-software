# Gives the current state of the complete collection of detectable scene nodes.
# Publishing all scene nodes in one messages can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

# The ID to assign to the next instantiated node
uint32 next_id

# A depth first ordered list of node indexes.
# The indexes are of the rest of the arrays in this message
# as if one continuous array ordered top to bottom
uint32[<=1000] scene_tree

# Basic scene nodes
perception_msgs/SceneNodeMessage[<=200] scene_nodes

# Detectable scene nodes
perception_msgs/DetectableSceneNodeMessage[<=200] detectable_scene_nodes

# ArUco marker scene nodes
perception_msgs/ArUcoMarkerNodeMessage[<=200] aruco_marker_scene_nodes

# Static relative scene nodes
perception_msgs/StaticRelativeSceneNodeMessage[<=200] static_relative_scene_nodes