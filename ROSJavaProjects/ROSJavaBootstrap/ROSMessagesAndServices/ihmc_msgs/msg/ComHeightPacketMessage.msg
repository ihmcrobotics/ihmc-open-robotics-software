## ComHeightPacketMessage
# This message sets the robot's center of mass height.

# heightOffset specifies CoM height relative to the default starting height, which is
# about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about
# 68.9 cm above ground level
float64 height_offset

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectory_time

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


