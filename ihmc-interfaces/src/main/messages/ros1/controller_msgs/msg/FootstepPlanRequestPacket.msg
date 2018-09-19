int8 FOOTSTEP_PLAN_REQUEST_TYPE_START_SEARCH=0

int8 FOOTSTEP_PLAN_REQUEST_TYPE_STOP_SEARCH=1

int8 FOOTSTEP_PLAN_REQUEST_TYPE_UPDATE_START=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

controller_msgs/FootstepDataMessage start_footstep

float64 theta_start

# Field default value 1.0
float64 max_sub_optimality

controller_msgs/FootstepDataMessage[] goals

# Field default value 255
int8 footstep_plan_request_type


