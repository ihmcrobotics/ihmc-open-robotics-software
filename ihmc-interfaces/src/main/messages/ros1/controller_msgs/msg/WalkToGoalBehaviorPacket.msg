int8 WALK_TO_GOAL_ACTION_FIND_PATH=0

int8 WALK_TO_GOAL_ACTION_EXECUTE=1

int8 WALK_TO_GOAL_ACTION_EXECUTE_UNKNOWN=2

int8 WALK_TO_GOAL_ACTION_STOP=3

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 walk_to_goal_action

float64 x_goal

float64 y_goal

float64 theta_goal

# Field default value 255
int8 goal_robot_side


