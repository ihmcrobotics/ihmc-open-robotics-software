# The base scene node information
perception_msgs/SceneNodeMessage scene_node
byte door_hardware_type
geometry_msgs/Pose object_pose
# The visual transform to parent
geometry_msgs/Transform visual_transform_to_object_pose
perception_msgs/PlanarRegionMessage door_planar_region
geometry_msgs/Point32[<=5000] presumed_door_point_cloud