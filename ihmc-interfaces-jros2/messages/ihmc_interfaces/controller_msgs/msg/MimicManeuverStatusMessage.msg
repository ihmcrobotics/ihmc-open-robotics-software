# Status of the robot-side AvatarMimic replay executor, for teleop button state.

byte PHASE_IDLE = 0
byte PHASE_EXECUTING_GROUND_PREP = 1
byte PHASE_REINITIALIZING_ESTIMATOR = 2
byte PHASE_SETTLING_OFFSET = 3
byte PHASE_SELECTING_MODEL = 4
byte PHASE_ENTERING_RL = 5
byte PHASE_REPLAYING = 6
byte PHASE_WAITING_POSTCONDITION = 7

byte COVER_STAGE_DOWN = 0
byte COVER_STAGE_UP = 1

byte phase
byte cover_stage
bool busy
string status_message
