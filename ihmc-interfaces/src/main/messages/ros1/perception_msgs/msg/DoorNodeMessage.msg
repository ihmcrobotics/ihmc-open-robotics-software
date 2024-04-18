
# The base scene node information
perception_msgs/SceneNodeMessage scene_node

# The type of door opening mechanism detected
int8 opening_mechanism_type

# The pose of the detected door hardware
geometry_msgs/Pose opening_mechanism_pose

# The planar region we assume is the door panel
perception_msgs/PlanarRegionMessage door_planar_region

# The last time the door planar region was updated
uint64 door_planar_region_update_time_millis


