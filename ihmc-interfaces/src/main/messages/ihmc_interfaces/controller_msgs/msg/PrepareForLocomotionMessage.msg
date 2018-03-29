# This message is part of the IHMC whole-body controller API.
# Configure the controller manipulation and pelvis managers.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
bool prepare_manipulation true
# When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
bool prepare_pelvis true