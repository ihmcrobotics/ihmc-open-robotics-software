# This message is part of the IHMC whole-body controller API.
# This message allows the user to precisely select what component in taskspace are to be achieved.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The ID of the reference frame defining the selection frame.
# When selecting the axes of interest, these axes refer to the selection frame axes.
# This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to 0,
# the selection matrix will then be generated regardless to what frame is it used in.
int64 selection_frame_id 0

# Specifies whether the x-axis of the selection frame is an axis of interest.
bool x_selected true
# Specifies whether the y-axis of the selection frame is an axis of interest.
bool y_selected true
# Specifies whether the z-axis of the selection frame is an axis of interest.
bool z_selected true