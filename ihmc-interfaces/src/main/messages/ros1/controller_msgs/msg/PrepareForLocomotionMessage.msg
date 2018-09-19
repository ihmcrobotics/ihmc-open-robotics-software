# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
# Field default value True
bool prepare_manipulation

# When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
# Field default value True
bool prepare_pelvis


