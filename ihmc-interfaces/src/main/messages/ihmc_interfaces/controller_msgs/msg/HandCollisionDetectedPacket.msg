# Message for the IHMC humanoid behavior module.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte robot_side 255
int32 collision_severity_level_one_to_three