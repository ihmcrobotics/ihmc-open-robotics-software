## SelectionMatrix3DRosMessage
# 

# The Id of the reference frame defining the selection frame. When selecting the axes of interest,
# these axes refer to the selection frame axes. This frame is optional. It is preferable to provide it
# when possible, but when it is absent, i.e. equal to {@code 0L}, the selection matrix will then be
# generated regardless to what frame is it used in.
int64 selection_frame_id

# Specifies whether the x-axis of the selection frame is an axis of interest.
bool x_selected

# Specifies whether the y-axis of the selection frame is an axis of interest.
bool y_selected

# Specifies whether the z-axis of the selection frame is an axis of interest.
bool z_selected

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


