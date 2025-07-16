# This message is part of the FootstepStreamingToolbox API

# Unique ID used to identify this message, should preferably be consecutively increasing
uint64 sequence_id

# Current robot step duration
float64 robot_step_duration

# Current robot swing duration
float64 robot_swing_duration

# Time elapsed since the robot has initiated this last step
float64 robot_step_elapsed_time

# Current swing foot side of the robot
byte robot_swing_side 255

# If the robot swing foot is landing. Foot is descending below max swing height
bool is_robot_swing_foot_landing

# The list of sides used as input for the toolbox to estimate footsteps for the robot
toolbox_msgs/FootstepStreamingToolboxSideMessage[<=2] side