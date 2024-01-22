# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte BODY_PATH_PLANNING_RESULT_PLANNING = 0
byte BODY_PATH_PLANNING_RESULT_FOUND_SOLUTION = 1
byte BODY_PATH_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = 2
byte BODY_PATH_PLANNING_RESULT_NO_PATH_EXISTS = 3
byte BODY_PATH_PLANNING_RESULT_INVALID_GOAL = 4
byte BODY_PATH_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED = 5
byte BODY_PATH_PLANNING_RESULT_EXCEPTION = 6
byte BODY_PATH_PLANNING_RESULT_HALTED = 7

byte FOOTSTEP_PLANNING_RESULT_PLANNING = 0
byte FOOTSTEP_PLANNING_RESULT_FOUND_SOLUTION = 1
byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = 2
byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = 3
byte FOOTSTEP_PLANNING_RESULT_INVALID_GOAL = 4
byte FOOTSTEP_PLANNING_RESULT_MAXIMUM_ITERATIONS_REACHED = 5
byte FOOTSTEP_PLANNING_RESULT_EXCEPTION = 6
byte FOOTSTEP_PLANNING_RESULT_HALTED = 7

int32 NO_PLAN_ID = -1

# ID of the request this output corresponds to
int32 plan_id -1

# Footstep plan, may be empty depending on the state of the planner, according to footstep_planning_result
controller_msgs/FootstepDataListMessage footstep_data_list

# Body path plan result. Null if no result is available
byte body_path_planning_result 255

# Footstep planner result. Null if no result is available
byte footstep_planning_result 255

# (deprecated) Height map that correspond to the request message. Originally used for debugging networking
perception_msgs/HeightMapMessage height_map_message

# Planned body path. Empty if planner failed
geometry_msgs/Pose[] body_path

# Planned body path before smoothing, used for debugging. Empty if planner failed
geometry_msgs/Point[] body_path_unsmoothed

# Goal pose used by the planner. This will be different from the requested goal pose if it's beyond the horizon length.
geometry_msgs/Pose goal_pose

# Object to record various planner timings, helpful for debugging
FootstepPlanningTimingsMessage planner_timings

# Contains planner stack trace if failure is due to an exception
string exception_message
string[<=20] stacktrace