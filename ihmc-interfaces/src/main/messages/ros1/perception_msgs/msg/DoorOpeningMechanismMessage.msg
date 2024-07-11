
# The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
int8 type

# The side of the door the opening mechanism is mounted to
# true  PUSH
# false PULL
bool door_side

# The transform of the opening mechanisms frame
geometry_msgs/Transform mechanism_transform_to_world

# ID of the persistent detection associated with this opening mechanism. May be null.
ihmc_common_msgs/UUIDMessage persistent_detection_id


