# This message is part of the IHMC whole-body controller API.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte NORMAL = 0
byte FROZEN = 1

byte requested_state_estimator_mode 255