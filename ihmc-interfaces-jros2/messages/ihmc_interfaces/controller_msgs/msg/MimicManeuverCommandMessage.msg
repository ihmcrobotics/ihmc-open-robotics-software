# Operator request to run an AvatarMimic recording on the robot controller.
# The UI only sends this button command; replay itself runs on the robot.

byte ACTION_NONE = 0
byte ACTION_STAND_UP = 1
byte ACTION_LIE_DOWN = 2
byte ACTION_COVER = 3
byte ACTION_RECEIVE_TROWEL = 4
byte ACTION_DIG = 5
byte ACTION_DIG_MORE = 6
byte ACTION_GET_UP = 7
byte ACTION_CANCEL = 8
byte ACTION_PICKUP_BUCKET = 9
byte ACTION_DROP_BUCKET = 10

byte requested_action
bool execute
