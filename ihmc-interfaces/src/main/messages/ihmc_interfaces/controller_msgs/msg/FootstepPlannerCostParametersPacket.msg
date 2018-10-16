# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When using a cost based planning approach this value defined how the yaw of a footstep will be
# weighted in comparison to its position.
float64 yaw_weight -1.0

# When using a cost based planning approach this value defines how the pitch will be weighted.
float64 pitch_weight -1.0

# When using a cost based planning approach this value defines how the roll will be weighted.
float64 roll_weight -1.0

# When using a cost based planning approach, this value defines how the forward (or backward) displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getLateralWeight()}
float64 forward_weight -1.0

# When using a cost based planning approach, this value defines how the lateral displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getForwardWeight()}
float64 lateral_weight -1.0

# When using a cost based planning approach this value defines how the height change when stepping up will be
# weighted.
float64 step_up_weight -1.0

# When using a cost based planning approach this value defines how the height change when stepping down will be
# weighted.
float64 step_down_weight -1.0

# When using a cost based planning approach this value defines the cost that is added for each step
# taken. Setting this value to a high number will favor plans with less steps.
float64 cost_per_step -1.0


