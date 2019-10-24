# This message is part of the IHMC whole-body controller API.
# This message notifies the user of a change in the high level state.
# This message's primary use is to signal a requested state change is completed.

byte DO_NOTHING_BEHAVIOR = 0
byte STAND_PREP_STATE = 1
byte STAND_READY = 2
byte FREEZE_STATE = 3
byte STAND_TRANSITION_STATE = 4
byte WALKING = 5
byte EXIT_WALKING = 6
byte DIAGNOSTICS = 7
byte CALIBRATION = 8
byte CUSTOM1 = 9
byte FALLING_STATE = 10

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the controller's state prior to transition.
byte initial_high_level_controller_name 255
# Specifies the state the controller has transitioned into.
byte end_high_level_controller_name 255