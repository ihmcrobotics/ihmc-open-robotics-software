# Atlas specific message.

byte NONE = 0
byte FREEZE = 1
byte STAND_PREP = 2
byte STAND = 3
byte WALK = 4
byte STEP = 5
byte MANIPULATE = 6
byte USER = 7
byte CALIBRATE = 8
byte SOFT_STOP = 9

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte current_bdi_robot_behavior 255