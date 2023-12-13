# This message is part of the IHMC whole-body controller API.
# This message specifies the position, orientation and side (left or right) of a footstep in the queue in world frame.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will swing to reach the footstep.
# Field default value 255
int8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location

# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation

# The swingDuration is the time a foot is not in ground contact during a step.
# If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
# Field default value -1.0
float64 swing_duration

# The transferDuration is the time spent with the feet in ground contact before a step.
# If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
# Field default value -1.0
float64 transfer_duration


