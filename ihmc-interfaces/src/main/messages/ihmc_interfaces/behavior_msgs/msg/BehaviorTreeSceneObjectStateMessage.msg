# A behavior tree scene object

# The timestamp and modifier ID of the latest modification to this object's data fields
ihmc_common_msgs/LatestModificationMessage latest_modification_to_data

# The object's unique ID
uint32 id

# Last 4 digits of persistent detection UUID as a string or "null"
string persistent_detection_id

# Object type (IsaacROSFoundationPoseObject ordinal)
int32 object_type

# Transform of the object frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world
