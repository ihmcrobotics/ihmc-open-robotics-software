# Operator request to run an AvatarMimic recording on the robot controller.
# The UI only sends this button command; replay itself runs on the robot.

byte ACTION_NONE = 0
byte ACTION_STAND_UP = 1
byte ACTION_LIE_DOWN = 2
byte ACTION_COVER = 3
byte ACTION_CANCEL = 8

byte requested_action
bool execute
