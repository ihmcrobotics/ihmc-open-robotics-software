# This message is part of the IHMC whole-body controller API.
# Configure the controller manipulation and pelvis managers.

# When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
bool prepare_manipulation true
# When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
bool prepare_pelvis true