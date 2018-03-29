# This message is part of the IHMC toolbox framework.

byte WAKE_UP = 0
byte REINITIALIZE = 1
byte SLEEP = 2

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte requested_toolbox_state 255