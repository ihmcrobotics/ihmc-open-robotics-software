# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte BODY_PATH_PLANNING_RESULT_FOUND_SOLUTION = 0
byte BODY_PATH_PLANNING_RESULT_NO_PATH_EXISTS = 1
byte BODY_PATH_PLANNING_RESULT_EXCEPTION = 2

byte FOOTSTEP_PLANNING_RESULT_PLANNING = 0
byte FOOTSTEP_PLANNING_RESULT_FOUND_SOLUTION = 1
byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = 2
byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = 3
byte FOOTSTEP_PLANNING_RESULT_INVALID_GOAL = 4
byte FOOTSTEP_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED = 5
byte FOOTSTEP_PLANNING_RESULT_EXCEPTION = 6
byte FOOTSTEP_PLANNING_RESULT_HALTED = 7

int32 NO_PLAN_ID = -1

FootstepDataListMessage footstep_data_list
byte body_path_planning_result 255
byte footstep_planning_result 255
int32 plan_id -1

PlanarRegionsListMessage planar_regions_list

geometry_msgs/Pose[] body_path
geometry_msgs/Pose low_level_planner_goal

FootstepPlanningTimingsMessage planner_timings

# Contains planner stack trace if failure is due to an exception
string exception_message
string[<=20] stacktrace