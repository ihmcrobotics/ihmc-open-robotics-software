# This message is part of the IHMC whole-body controller API.
# Configure the controller manipulation and pelvis managers.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
bool prepare_manipulation true
# When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
bool prepare_pelvis true