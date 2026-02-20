# A behavior tree scene object

# The timestamp and modifier ID of the latest modification to this object's data fields
ihmc_common_msgs/LatestModificationMessage latest_modification_to_data

# The object's unique ID
uint32 id

behavior_msgs/BehaviorTreeSceneObjectDefinitionMessage definition

# Persistent detection associated with this object
behavior_msgs/PersistentDetectionStatusMessage persistent_detection

# Transform of the object frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world

# Used only for door panel
behavior_msgs/PersistentDetectionStatusMessage door_panel_detection

# Used only for door frame
uint32 points_in_capsule

# The open angle of the door hinge (0 = closed, Zup yaw)
float32 door_open_angle

byte DOOR_TYPE_UNKNOWN = 0
byte DOOR_TYPE_PUSH = 1
byte DOOR_TYPE_PULL = 2

# Door type as defined above
byte door_type

# Whether the object is frozen
bool frozen
