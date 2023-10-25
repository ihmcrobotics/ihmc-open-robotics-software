# An perception scene node for an ArUco marker
# The topic name identifies the node.

# The detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# Object ID
int32 object_id

# Position and Orientation of the object
geometry_msgs/Pose pose

# How confident are we about what object it is. Mostly used to NN based detections
float64 confidence

# Object Category
string object_type

# 2D Vertices of the 3d object bounding box projected onto image plane
geometry_msgs/Point[] bounding_box_2d_vertices

# 3d Vertices of the 3d object Bounding box
geometry_msgs/Point[] bounding_box_vertices

# Break frequency filter value for nodes that are alpha filtered
float32 break_frequency


