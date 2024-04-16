# The base scene node information
perception_msgs/SceneNodeMessage scene_node
# The type of door opening mechanism detected
byte opening_mechanism_type
# The pose of the detected door hardware
geometry_msgs/Pose opening_mechanism_pose
# The visual transform to parent of the door hardware
geometry_msgs/Transform opening_mechanism_visual_transform_to_object_pose
# The planar region we assume is the door panel
perception_msgs/PlanarRegionMessage door_planar_region