# This message carries robot's joints, IMU, and force sensors' reference frame information.
# It is published frequently from the IHMC state estimator along with robot configuration data.

# Platform dependent.
# Time signal in nanoseconds that can be used to synchronize two time sources.
int64 timestamp

# Hash code of this frame.
int32 frame_name_hash

# Pose3D of this frame in the world frame.
geometry_msgs/Pose frame_pose_in_world

# Name of this frame.
string frame_name