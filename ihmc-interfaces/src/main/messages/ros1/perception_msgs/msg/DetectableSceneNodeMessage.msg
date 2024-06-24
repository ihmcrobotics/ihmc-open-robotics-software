# A detectable perception scene node
# The topic name identifies the node.

# The base scene node information
perception_msgs/SceneNodeMessage scene_node

# Instant detection information
perception_msgs/InstantDetectionMessage[] latest_detections

# Whether or not the node is currently detected
bool currently_detected


