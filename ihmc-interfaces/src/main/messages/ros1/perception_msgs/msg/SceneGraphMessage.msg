# Gives the current state of the complete collection of detectable scene nodes.
# Publishing all scene nodes in one messages can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

int8 SCENE_NODE_TYPE=0

int8 DETECTABLE_SCENE_NODE_TYPE=1

int8 PREDEFINED_RIGID_BODY_NODE_TYPE=2

int8 ARUCO_MARKER_NODE_TYPE=3

int8 CENTERPOSE_NODE_TYPE=4

int8 STATIC_RELATIVE_NODE_TYPE=5

int8 PRIMITIVE_RIGID_BODY_NODE_TYPE=6

# The ID to assign to the next instantiated node
uint32 next_id

# A depth first ordered list of types.
int8[] scene_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[] scene_tree_indices

# Basic scene nodes
perception_msgs/SceneNodeMessage[] scene_nodes

# Detectable scene nodes
perception_msgs/DetectableSceneNodeMessage[] detectable_scene_nodes

# Predefined rigid body scene nodes
perception_msgs/PredefinedRigidBodySceneNodeMessage[] predefined_rigid_body_scene_nodes

# ArUco marker scene nodes
perception_msgs/ArUcoMarkerNodeMessage[] aruco_marker_scene_nodes

# Centerpose scene nodes
perception_msgs/CenterposeNodeMessage[] centerpose_scene_nodes

# Static relative scene nodes
perception_msgs/StaticRelativeSceneNodeMessage[] static_relative_scene_nodes

# Reshapable rigid body scene nodes
perception_msgs/PrimitiveRigidBodySceneNodeMessage[] primitive_rigid_body_scene_nodes


