int8 FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION=0

int8 FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION=1

int8 FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION=2

int8 FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS=3

int8 FOOTSTEP_PLANNING_RESULT_SNAPPING_FAILED=4

int8 FOOTSTEP_PLANNING_RESULT_PLANNER_FAILED=5

int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

controller_msgs/FootstepDataListMessage footstep_data_list

# Field default value 255
int8 footstep_planning_result

# Field default value -1
int32 plan_id

controller_msgs/PlanarRegionsListMessage planar_regions_list

geometry_msgs/Point[] body_path

geometry_msgs/Pose low_level_planner_goal

controller_msgs/FootstepPlanningStatistics footstep_planning_statistics


