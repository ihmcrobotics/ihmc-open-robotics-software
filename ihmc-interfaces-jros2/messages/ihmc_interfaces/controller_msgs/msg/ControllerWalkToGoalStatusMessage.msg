# Status message for the PDVelocityBasedGoalReacher.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Current pose for the robot to reach.
float64 current_goal_x_position
float64 current_goal_y_position
float64 current_goal_yaw

geometry_msgs/Point current_position

bool is_reached
