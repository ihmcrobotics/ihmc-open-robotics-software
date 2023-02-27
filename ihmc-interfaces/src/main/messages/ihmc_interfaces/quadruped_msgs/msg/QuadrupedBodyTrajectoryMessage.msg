# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the body to the desired pose (position & orientation) while going through the specified trajectory points.
# A third order polynomial function is used to interpolate positions and a Hermite based curve (third order) is used to interpolate the orientations.
# To execute a single straight line trajectory to reach a desired body pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
# Note that the body position is limited keep the robot's balance (center of mass has to remain inside the support polygon).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time true
# The position/orientation trajectory information.
ihmc_common_msgs/SE3TrajectoryMessage se3_trajectory