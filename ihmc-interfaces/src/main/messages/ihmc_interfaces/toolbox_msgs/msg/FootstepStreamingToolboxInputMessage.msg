# This message is part of the FootstepStreamingToolbox API

# Unique ID used to identify this message, should preferably be consecutively increasing
uint64 sequence_id

# Current robot step duration
float64 robot_step_duration

# Time elapsed since the robot has initiated this last step
float64 robot_step_elapsed_time

# The list of sides used as input for the toolbox to estimate footsteps for the robot
toolbox_msgs/FootstepStreamingToolboxSideMessage[<=2] side