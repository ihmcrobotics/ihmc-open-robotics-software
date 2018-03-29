# This message is used to switch the control scheme between force and position control.
# WARNING: When in position control, the IHMC balance algorithms will be disabled and it is up to the user to ensure stability.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte exoskeleton_behavior_state