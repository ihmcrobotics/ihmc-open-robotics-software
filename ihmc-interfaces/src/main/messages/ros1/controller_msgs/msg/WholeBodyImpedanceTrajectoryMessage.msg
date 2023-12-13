# This message is part of the whole-body impedance controller API.
# Allows to send joint-space trajectory along with a timed contact sequence which determines limb stiffness.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The is the unique hash code of each of the joints to be controlled.
# It is used on the controller side to retrieve the desired joint to be controlled.
# See Joint.hashCode() for the computation of a joint hash code.
int32[] joint_hash_codes

# Trajectory for each joint.
controller_msgs/OneDoFJointTrajectoryMessage[] joint_trajectory_messages

# Properties for queueing trajectories.
ihmc_common_msgs/QueueableMessage queueing_properties

# Contact sequence message
controller_msgs/MultiContactTimedContactSequenceMessage contact_sequence_message


