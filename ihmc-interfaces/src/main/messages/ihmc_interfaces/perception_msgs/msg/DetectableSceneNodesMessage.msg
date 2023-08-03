# Gives the current state of the complete collection of detectable scene nodes.
# Publishing all scene nodes in one messages can simplify synchronization and
# reduce the complexity of logic in figuring out when nodes are currently under
# consideration.

# The current state of the complete collection of detectable scene nodes
perception_msgs/DetectableSceneNodeMessage[<=5000] detectable_scene_nodes