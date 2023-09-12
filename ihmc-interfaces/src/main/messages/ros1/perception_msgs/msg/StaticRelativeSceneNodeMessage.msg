# A static relative heuristic scene node
# The topic name identifies the node.

# The predefined rigid body information that this extends
perception_msgs/PredefinedRigidBodySceneNodeMessage predefined_rigid_body_scene_node

# Distance to robot to disable tracking.
# Used for automatically disabling tracking for static relative objects.
float32 distance_to_disable_tracking

# Current distance to robot.
# Used for automatically disabling tracking for static relative objects.
float32 current_distance_to_robot


