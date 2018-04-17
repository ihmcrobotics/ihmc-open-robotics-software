# This message is part of the IHMC quadruped controller API.
# This message notifies the user of a change in the high level controller state.
# This message's primary use is to signal a requested state change is completed.

byte JOINT_INITIALIZATION = 0
byte DO_NOTHING = 1
byte STAND_PREP = 2
byte STAND_READY = 3
byte FREEZE = 4
byte STEPPING = 5
byte FALL = 6

# Specifies the controller's state prior to transition.
byte initial_controller_name 255
# Specifies the state the controller has transitioned into.
byte end_controller_name 255