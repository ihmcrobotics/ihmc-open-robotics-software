# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the pelvis to the desired pose (position & orientation) while going through the specified trajectory points.
# A third order polynomial function is used to interpolate positions and a Hermite based curve (third order) is used to interpolate the orientations.
# To excute a single straight line trajectory to reach a desired pelvis pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
# Note that the pelvis position is limited keep the robot's balance (center of mass has to remain inside the support polygon).

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
bool enable_user_pelvis_control false
# If enable_user_pelvis_control is true then enable_user_pelvis_control_during_walking
# will keep the manager in user mode while walking.
# If this is false the manager will switch back to controller mode when walking.
bool enable_user_pelvis_control_during_walking false
# The position/orientation trajectory information.
SE3TrajectoryMessage se3_trajectory