# This message is part of the Quix controller API.
# This message is used to allow the crutch to communicate a desired change in behavior.

uint32 sequence_id

bool user_enable

bool rewiggle

bool start_behavior

exoskeleton_msgs/QuixMotionStateMessage requested_motion_state

bool execute_behavior

bool continuous_walking

exoskeleton_msgs/FlatStepTypeMessage flat_step_type

exoskeleton_msgs/QuixStairsStepTypeMessage stairs_step_type

exoskeleton_msgs/QuixSideStepDirectionMessage side_step_direction

exoskeleton_msgs/QuixSlopeStepTypeMessage slope_step_type

# Field default value 255
int8 force_swing_side


