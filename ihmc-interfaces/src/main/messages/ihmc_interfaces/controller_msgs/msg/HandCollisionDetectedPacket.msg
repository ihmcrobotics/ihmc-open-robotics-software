# Message for the IHMC humanoid behavior module.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte robot_side 255
int32 collision_severity_level_one_to_three