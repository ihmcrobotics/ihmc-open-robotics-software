# This message is part of the IHMC multi-contact controller API.
# Experimental mode of specifying a desired configuration in terms of root pose and joint angles.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Duration of trajectory in seconds
float64 trajectory_duration
# Target joint angles of the trajectory
float64[<=50] joint_angles
# Terminal root joint pose in world frame
geometry_msgs/Pose root_joint_pose
# Hash of joint array
int32 joint_name_hash