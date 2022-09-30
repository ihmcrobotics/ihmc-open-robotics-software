# This message is part of the IHMC humanoid behavior module.

int8 UNKNOWN_TYPE=0

int8 PUSH_HANDLE_LEFT=1

int8 PUSH_HANDLE_RIGHT=2

int8 PULL_HANDLE_LEFT=3

int8 PULL_HANDLE_RIGHT=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Pose door_transform_to_world

# Field default value 255
int8 detected_door_type

# Some behaviors will average the door location to remove noise,
# trustedPosition allows someone to tell the behavior that this is an accurate location and and can does not need to be averaged,
# this is useful for sending door locations manually from the ui*/
bool trusted_position


