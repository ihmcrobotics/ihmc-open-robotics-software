# This message is part of the Quix controller API.
# This message is used to allow the crutch to communicate a desired change in behavior.

uint32 sequence_id

bool user_enable false
bool rewiggle false
bool start_behavior false
controller_msgs/QuixMotionStateMessage requested_motion_state
bool execute_behavior false