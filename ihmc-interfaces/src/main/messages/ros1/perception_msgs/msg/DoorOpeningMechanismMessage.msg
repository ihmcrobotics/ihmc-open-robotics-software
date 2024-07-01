
# The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
int8 type

# The side of the door the opening mechanism is mounted to
# true  PUSH
# false PULL
bool door_side

# The grasp pose of the opening mechanism
geometry_msgs/Pose mechanism_pose

# ID of the persistent detection associated with this opening mechanism. May be null.
ihmc_common_msgs/UUIDMessage persistent_detection_id


