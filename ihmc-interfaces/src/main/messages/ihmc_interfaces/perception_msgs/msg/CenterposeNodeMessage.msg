# An perception scene node for an ArUco marker
# The topic name identifies the node.

# The detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

string object_type
int32 object_id
float64 confidence
geometry_msgs/Point[8] bounding_box_vertices
geometry_msgs/Point[8] bounding_box_vertices_2d

bool enable_tracking