# Atlas specific message.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

byte ATLAS_LOW_LEVEL_CONTROL_MODE_STAND_PREP = 0
byte ATLAS_LOW_LEVEL_CONTROL_MODE_FREEZE = 1

byte requested_atlas_low_level_control_mode 255