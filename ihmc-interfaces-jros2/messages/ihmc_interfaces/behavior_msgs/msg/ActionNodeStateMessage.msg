# Parent state fields
behavior_msgs/LeafNodeStateMessage state

# Nominal execution duration
float64 nominal_execution_duration

# Time since execution started
float64 elapsed_execution_time

# Desired trajectory
ihmc_common_msgs/SE3TrajectoryPointMessage[<=500] commanded_trajectory

# Current pose
geometry_msgs/Pose current_pose

# Desired jointspace trajectories
controller_msgs/OneDoFJointTrajectoryMessage[<=7] commanded_joint_trajectories

# Current joint angles for tracking jointspace trajectories
float64[7] current_joint_angles

# Position distance to goal tolerance
float64 position_distance_to_goal_tolerance

# Orientation distance to goal tolerance
float64 orientation_distance_to_goal_tolerance
