# This message is part of the IHMC whole-body controller API.
# Experimental mode of specifying a desired configuration in terms of root pose and joint angles.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 trajectory_duration

float64[] joint_angles

geometry_msgs/Pose pelvis_pose

int32 joint_name_hash


