# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the pelvis to the desired pose (position & orientation) while going through the specified trajectory points.
# A third order polynomial function is used to interpolate positions and a Hermite based curve (third order) is used to interpolate the orientations.
# To excute a single straight line trajectory to reach a desired pelvis pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
# Note that the pelvis position is limited keep the robot's balance (center of mass has to remain inside the support polygon).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution

# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
bool enable_user_pelvis_control

# If enable_user_pelvis_control is true then enable_user_pelvis_control_during_walking
# will keep the manager in user mode while walking.
# If this is false the manager will switch back to controller mode when walking.
bool enable_user_pelvis_control_during_walking

# The position/orientation trajectory information.
controller_msgs/SE3TrajectoryMessage se3_trajectory


