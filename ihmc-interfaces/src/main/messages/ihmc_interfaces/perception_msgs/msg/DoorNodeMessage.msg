# The base scene node information
perception_msgs/SceneNodeMessage scene_node

# The type of door opening mechanism detected
byte opening_mechanism_type

# The 3D point of the detected door hardware
geometry_msgs/Point opening_mechanism_point

# The pose of the detected door hardware wrt the door planar region
geometry_msgs/Pose opening_mechanism_pose

# The planar region we assume is the door panel
perception_msgs/PlanarRegionMessage door_planar_region

# The last time the door planar region was updated
uint64 door_planar_region_update_time_millis