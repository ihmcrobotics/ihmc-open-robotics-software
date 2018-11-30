# Atlas specific message: request taring of the wrist force/torque sensors.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The robot side (left or right) for the wrist sensor you would like to request calibration for.
byte robot_side 255
