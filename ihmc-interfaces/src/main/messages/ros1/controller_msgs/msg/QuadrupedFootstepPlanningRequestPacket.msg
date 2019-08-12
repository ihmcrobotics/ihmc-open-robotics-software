int8 ROBOT_QUADRANT_FRONT_LEFT=0

int8 ROBOT_QUADRANT_FRONT_RIGHT=1

int8 ROBOT_QUADRANT_HIND_LEFT=2

int8 ROBOT_QUADRANT_HIND_RIGHT=3

int8 FOOTSTEP_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET=0

int8 FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS=1

int32 NO_PLAN_ID=-1

int8 FOOTSTEP_PLANNER_TYPE_SIMPLE_PATH_TURN_WALK_TURN=0

int8 FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_TURN_WALK_TURN=1

int8 FOOTSTEP_PLANNER_TYPE_A_STAR=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 initial_step_robot_quadrant

int8 start_target_type

geometry_msgs/Point body_position_in_world

geometry_msgs/Quaternion body_orientation_in_world

geometry_msgs/Point front_left_position_in_world

geometry_msgs/Point front_right_position_in_world

geometry_msgs/Point hind_left_position_in_world

geometry_msgs/Point hind_right_position_in_world

geometry_msgs/Point goal_position_in_world

geometry_msgs/Quaternion goal_orientation_in_world

# Field default value -1
int32 planner_request_id

# Field default value 255
int8 requested_footstep_planner_type

float64 timeout

float64 best_effort_timeout

float64 horizon_length

controller_msgs/PlanarRegionsListMessage planar_regions_list_message

bool assume_flat_ground


