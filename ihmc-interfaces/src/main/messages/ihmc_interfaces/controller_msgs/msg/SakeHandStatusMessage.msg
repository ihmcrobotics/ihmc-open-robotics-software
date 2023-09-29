# Message used to report the current joint angles for the fingers of the sake gripper.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte robot_side 255
float64 temperature
# 0.0 min, 1.0 max
float64 present_torque_ratio
# 0.0 closed, 1.0 open
float64 postion_ratio
# 0.0 min, 1.0 max
float64 goal_torque_ratio

bool calibrated
bool needs_reset