# (Obsolete) This message is part of the old IHMC footstep planning module.

byte FOOTSTEP_PLAN_REQUEST_TYPE_START_SEARCH = 0;
byte FOOTSTEP_PLAN_REQUEST_TYPE_STOP_SEARCH = 1;
byte FOOTSTEP_PLAN_REQUEST_TYPE_UPDATE_START = 2;

FootstepDataMessage start_footstep
float64 theta_start
float64 max_sub_optimality 1.0
FootstepDataMessage[] goals
byte footstep_plan_request_type -1