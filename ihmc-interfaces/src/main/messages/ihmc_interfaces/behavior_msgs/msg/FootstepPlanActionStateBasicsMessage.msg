# Total number of footsteps; used for walking actions
uint16 total_number_of_footsteps

# Incomplete footsteps; used for walking actions
uint16 number_of_incomplete_footsteps

# Desired left footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[<=50] desired_left_footsteps

# Desired right footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[<=50] desired_right_footsteps

# Current left pose
geometry_msgs/Pose current_left_foot_pose

# Current right pose
geometry_msgs/Pose current_right_foot_pose