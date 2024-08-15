# This message is part of the IHMC whole-body controller API.
# This message is used to trigger a donkey (back) kick. The robot should be in an appropriate position to execute the kick.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the kick.
byte robot_side

# The height at which the kick should be targeted.
float64 kick_height

# The impulse with which the kick should be executed.
float64 kick_impulse

# The target distance from the robot to where the kick should be aimed.
float64 kick_target_distance

# A boolean for tracking whether a kick has been requested.
bool trigger_kick_request

# Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
float64 prekick_weight_distribution