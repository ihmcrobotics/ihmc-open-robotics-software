# This message is part of the IHMC humanoid behavior module.

byte NO_BEHAVIOR_RUNNING = 0
byte BEHAVIOS_RUNNING = 1
byte BEHAVIOR_PAUSED = 2

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte current_behavior_status 255