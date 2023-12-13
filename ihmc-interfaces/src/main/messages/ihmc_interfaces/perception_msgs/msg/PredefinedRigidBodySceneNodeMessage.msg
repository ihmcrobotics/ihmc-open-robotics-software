# A predefined rigid body scene node
# The topic name identifies the node.

# The base scene node information
perception_msgs/SceneNodeMessage scene_node

# Initial parent ID
uint32 initial_parent_id

# Initial transform to initial parent
controller_msgs/RigidBodyTransformMessage initial_transform_to_parent

# The path to the visual model file
string visual_model_file_path

# The visual transform to parent
controller_msgs/RigidBodyTransformMessage visual_transform_to_parent
