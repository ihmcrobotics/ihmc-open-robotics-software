# An perception scene node for an ArUco marker
# The topic name identifies the node.

# The detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# 2D Vertices of the 3d object bounding box projected onto image plane
geometry_msgs/Point[] bounding_box_2d_vertices

# 3d Vertices of the 3d object Bounding box
geometry_msgs/Point[] bounding_box_vertices

bool enable_tracking


