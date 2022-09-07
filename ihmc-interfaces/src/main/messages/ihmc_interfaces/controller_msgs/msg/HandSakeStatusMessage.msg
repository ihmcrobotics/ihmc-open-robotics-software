# Message used to report the current joint angles for the fingers of a hand.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte robot_side 255
float64 temperature
float64 torque_ratio
float64 postion_ratio

bool calibrated
bool needs_reset