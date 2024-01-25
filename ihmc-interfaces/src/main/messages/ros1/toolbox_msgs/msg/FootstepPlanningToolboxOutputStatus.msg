# This message is part of the IHMC footstep planning module.

int8 BODY_PATH_PLANNING_RESULT_PLANNING=0

int8 BODY_PATH_PLANNING_RESULT_FOUND_SOLUTION=1

int8 BODY_PATH_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION=2

int8 BODY_PATH_PLANNING_RESULT_NO_PATH_EXISTS=3

int8 BODY_PATH_PLANNING_RESULT_INVALID_GOAL=4

int8 BODY_PATH_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED=5

int8 BODY_PATH_PLANNING_RESULT_EXCEPTION=6

int8 BODY_PATH_PLANNING_RESULT_HALTED=7

int8 FOOTSTEP_PLANNING_RESULT_PLANNING=0

int8 FOOTSTEP_PLANNING_RESULT_FOUND_SOLUTION=1

int8 FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION=2

int8 FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS=3

int8 FOOTSTEP_PLANNING_RESULT_INVALID_GOAL=4

int8 FOOTSTEP_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED=5

int8 FOOTSTEP_PLANNING_RESULT_EXCEPTION=6

int8 FOOTSTEP_PLANNING_RESULT_HALTED=7

int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# ID of the request this output corresponds to
# Field default value -1
int32 plan_id

# Footstep plan, may be empty depending on the state of the planner, according to footstep_planning_result
controller_msgs/FootstepDataListMessage footstep_data_list

# Body path plan result. Null if no result is available
# Field default value 255
int8 body_path_planning_result

# Footstep planner result. Null if no result is available
# Field default value 255
int8 footstep_planning_result

# (deprecated) Height map that correspond to the request message. Originally used for debugging networking
perception_msgs/HeightMapMessage height_map_message

# Planned body path. Empty if planner failed
geometry_msgs/Pose[] body_path

# Planned body path before smoothing, used for debugging. Empty if planner failed
geometry_msgs/Point[] body_path_unsmoothed

# Goal pose used by the planner. This will be different from the requested goal pose if it's beyond the horizon length.
geometry_msgs/Pose goal_pose

# Object to record various planner timings, helpful for debugging
toolbox_msgs/FootstepPlanningTimingsMessage planner_timings

# Contains planner stack trace if failure is due to an exception
string exception_message

string[] stacktrace


