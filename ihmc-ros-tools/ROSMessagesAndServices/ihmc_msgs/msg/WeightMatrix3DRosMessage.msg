## WeightMatrix3DRosMessage
# 

# The Id of the reference frame defining the weight frame. This reference frame defines the x axis,y
# axis, z axis for the weights. This frame is optional. It is preferable to provide it when possible,
# but when it is absent, i.e. equal to {@code 0L}, the weight matrix will then be generated regardless
# to what frame is it used in.
int64 weight_frame_id

# Specifies the qp weight for the x-axis, If set to NaN the controller will use the default weight for
# this axis. The weight is NaN by default.
float64 x_weight

# Specifies the qp weight for the y-axis, If set to NaN the controller will use the default weight for
# this axis. The weight is NaN by default.
float64 y_weight

# Specifies the qp weight for the z-axis, If set to NaN the controller will use the default weight for
# this axis. The weight is NaN by default.
float64 z_weight

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


