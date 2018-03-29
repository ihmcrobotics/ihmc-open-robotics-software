# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move the pelvis to a new height in the trajectory frame while going through the specified trajectory points.
# Sending this command will not affect the pelvis horizontal position.
# To control the pelvis 3D position use the PelvisTrajectoryMessage instead.

# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
bool enable_user_pelvis_control false
# If enable_user_pelvis_control is true then enable_user_pelvis_height_control_during_walking
# will keep the height manager in user mode while walking.
# If this is false the height manager will switch back to controller mode when walking.
bool enable_user_pelvis_control_during_walking false
# The position trajectory information.
EuclideanTrajectoryMessage euclidean_trajectory