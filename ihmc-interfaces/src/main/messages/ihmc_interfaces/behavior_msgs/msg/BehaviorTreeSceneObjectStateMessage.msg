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

# Whether the object is frozen
bool frozen

# Used only for door panel
behavior_msgs/PersistentDetectionStatusMessage door_panel_detection

# Number of points in the latch post detection capsule
uint32 latch_post_points

# Number of points in the hinge side recess detection capsule
uint32 hinge_recess_points

# The open angle of the door hinge (0 = closed, Zup yaw)
float32 door_open_angle

byte DOOR_TYPE_UNKNOWN = 0
byte DOOR_TYPE_PUSH = 1
byte DOOR_TYPE_PULL = 2

# Door type as defined above
byte door_type

# Hinge side from the view of the robot
byte hinge_side

# Table approach

uint32 left_table_points

uint32 right_table_points

geometry_msgs/Point left_capsule_center

geometry_msgs/Point right_capsule_center
