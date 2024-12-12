# This message is part of the IHMC whole-body controller API.
# This class is used to change the PID gains over time.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of trajectory points (in taskpsace) to go through while executing the trajectory.
SE3PIDGainsTrajectoryPointMessage[<=200] pid_gains_trajectory_points
# Properties for queueing trajectories.
ihmc_common_msgs/QueueableMessage queueing_properties