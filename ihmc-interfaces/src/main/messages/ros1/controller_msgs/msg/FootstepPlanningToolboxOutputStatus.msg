# This message is part of the IHMC footstep planning module.

int8 BODY_PATH_PLANNING_RESULT_FOUND_SOLUTION=0

int8 BODY_PATH_PLANNING_RESULT_NO_PATH_EXISTS=1

int8 BODY_PATH_PLANNING_RESULT_EXCEPTION=2

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

controller_msgs/FootstepDataListMessage footstep_data_list

# Field default value 255
int8 body_path_planning_result

# Field default value 255
int8 footstep_planning_result

# Field default value -1
int32 plan_id

controller_msgs/PlanarRegionsListMessage planar_regions_list

geometry_msgs/Pose[] body_path

geometry_msgs/Pose low_level_planner_goal

controller_msgs/FootstepPlanningTimingsMessage planner_timings

# Contains planner stack trace if failure is due to an exception
string exception_message

string[] stacktrace


