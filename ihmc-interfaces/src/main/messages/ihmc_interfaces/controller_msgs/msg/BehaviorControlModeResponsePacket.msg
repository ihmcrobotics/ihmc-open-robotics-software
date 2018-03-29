# This message is part of the IHMC humanoid behavior module.

byte STOP = 0
byte PAUSE = 1
byte RESUME = 2

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte behavior_control_mode_enum_request 255