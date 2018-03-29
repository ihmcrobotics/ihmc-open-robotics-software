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

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte atlas_bdi_robot_behavior 255
bool stop false