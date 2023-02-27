# This message is part of the IHMC quadruped controller API.
# This message notifies the user of a change in the stepping controller state.
# This message's primary use is to signal a requested state change is completed.

byte STAND = 0
byte STEP = 1
byte SOLE_WAYPOINT = 2

# Specifies the controller's state prior to transition.
byte initial_quadruped_stepping_state_enum 255
# Specifies the state the controller has transitioned into.
byte end_quadruped_stepping_state_enum 255