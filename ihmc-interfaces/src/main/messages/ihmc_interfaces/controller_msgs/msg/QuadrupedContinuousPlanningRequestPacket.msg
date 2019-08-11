# This message is part of the IHMC quadruped footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte FOOTSTEP_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET = 0
byte FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS = 1

int32 UNSET_ID = -1

byte start_target_type 0
geometry_msgs/Point goal_position_in_world
geometry_msgs/Quaternion goal_orientation_in_world
int32 planner_request_id -1
float64 timeout
float64 horizon_length
