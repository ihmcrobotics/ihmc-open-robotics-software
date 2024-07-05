# The base scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# The transform of the frame of the door corner (X points in the direction the door opens)
geometry_msgs/Transform door_corner_transform_to_world
# Whether the pose of the door frame is locked
bool pose_locked

# The door panel
perception_msgs/DoorPanelMessage door_panel

# The door opening mechanisms
perception_msgs/DoorOpeningMechanismMessage[] opening_mechanisms