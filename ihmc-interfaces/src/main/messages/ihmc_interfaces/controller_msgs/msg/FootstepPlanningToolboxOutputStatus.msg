# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION = 0
byte FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION = 1
byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = 2
byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = 3
byte FOOTSTEP_PLANNING_RESULT_SNAPPING_FAILED = 4
byte FOOTSTEP_PLANNING_RESULT_PLANNER_FAILED = 5
byte FOOTSTEP_PLANNING_RESULT_INVALID_GOAL = 6

int32 NO_PLAN_ID = -1

FootstepDataListMessage footstep_data_list
byte footstep_planning_result 255
int32 plan_id -1

PlanarRegionsListMessage planar_regions_list

geometry_msgs/Point[] body_path
geometry_msgs/Pose low_level_planner_goal

FootstepPlanningStatistics footstep_planning_statistics