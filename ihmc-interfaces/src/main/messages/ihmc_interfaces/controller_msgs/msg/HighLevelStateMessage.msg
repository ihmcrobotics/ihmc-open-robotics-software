# This message is part of the IHMC whole-body controller API.
# This message is used to switch the control scheme between different control mode.

byte DO_NOTHING_BEHAVIOR = 0
byte STAND_PREP_STATE = 1
byte STAND_READY = 2
byte FREEZE_STATE = 3
byte STAND_TRANSITION_STATE = 4
byte WALKING = 5
byte DIAGNOSTICS = 6
byte CALIBRATION = 7

# Specifies the which state the controller should transition into.
byte high_level_controller_name 255
