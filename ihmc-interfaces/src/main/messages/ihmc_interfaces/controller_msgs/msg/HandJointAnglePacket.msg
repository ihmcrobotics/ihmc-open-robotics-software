# Message used to report the current joint angles for the fingers of a hand.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte robot_side 255
float64[] joint_angles
bool connected
bool calibrated
