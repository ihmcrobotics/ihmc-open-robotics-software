# This message is part of the Quix controller API.
# This message is used to allow the crutch to communicate a desired change in behavior.

uint32 sequence_id

bool user_enable false
bool rewiggle false
bool start_behavior false
QuixMotionStateMessage requested_motion_state
bool execute_behavior false
bool continuous_walking false
FlatStepTypeMessage flat_step_type
QuixStairsStepTypeMessage stairs_step_type
QuixSideStepDirectionMessage side_step_direction
QuixSlopeStepTypeMessage slope_step_type
byte force_swing_side 255