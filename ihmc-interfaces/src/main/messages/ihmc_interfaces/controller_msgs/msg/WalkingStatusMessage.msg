# This message is part of the IHMC whole-body controller API.
# The controller sends this message to notify the user of the status of walking.

byte STARTED = 0
byte COMPLETED = 1
byte ABORT_REQUESTED = 2

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Status of walking.
byte walking_status 255