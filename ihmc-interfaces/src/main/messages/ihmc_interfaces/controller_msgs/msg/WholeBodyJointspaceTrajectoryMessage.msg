# This message is part of the IHMC whole-body controller API.
# Allows to send joint-space trajectory to either part of the joints or for all the joints.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The is the unique hash code of each of the joints to be controlled.
# It is used on the controller side to retrieve the desired joint to be controlled.
# See Joint.hashCode() for the computation of a joint hash code.
int32[] joint_hash_codes
# Trajectory for each joint.
OneDoFJointTrajectoryMessage[] joint_trajectory_messages
# Properties for queueing trajectories.
QueueableMessage queueing_properties