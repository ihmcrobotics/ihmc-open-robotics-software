# Gives the current state of the complete collection of detectable scene nodes.
# Publishing all scene nodes in one messages can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

byte SCENE_NODE_TYPE=0
byte DETECTABLE_SCENE_NODE_TYPE=1
byte PREDEFINED_RIGID_BODY_NODE_TYPE=2
byte ARUCO_MARKER_NODE_TYPE=3
byte CENTERPOSE_NODE_TYPE=4
byte STATIC_RELATIVE_NODE_TYPE=5
byte PRIMITIVE_RIGID_BODY_NODE_TYPE=6

# The ID to assign to the next instantiated node
uint32 next_id

# A depth first ordered list of types.
byte[<=1000] scene_tree_types

# A depth first ordered list of node indexes.
# The index is of that node in it's respective list for
# it's type.
uint32[<=1000] scene_tree_indices

# Basic scene nodes
perception_msgs/SceneNodeMessage[<=200] scene_nodes

# Detectable scene nodes
perception_msgs/DetectableSceneNodeMessage[<=200] detectable_scene_nodes

# Predefined rigid body scene nodes
perception_msgs/PredefinedRigidBodySceneNodeMessage[<=200] predefined_rigid_body_scene_nodes

# ArUco marker scene nodes
perception_msgs/ArUcoMarkerNodeMessage[<=200] aruco_marker_scene_nodes

# Centerpose scene nodes
perception_msgs/CenterposeNodeMessage[<=200] centerpose_scene_nodes

# Static relative scene nodes
perception_msgs/StaticRelativeSceneNodeMessage[<=200] static_relative_scene_nodes

# Reshapable rigid body scene nodes
perception_msgs/PrimitiveRigidBodySceneNodeMessage[<=200] primitive_rigid_body_scene_nodes