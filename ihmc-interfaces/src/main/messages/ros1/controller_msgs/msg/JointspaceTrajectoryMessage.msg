# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Trajectory for each joint.
controller_msgs/OneDoFJointTrajectoryMessage[] joint_trajectory_messages

# Properties for queueing trajectories.
controller_msgs/QueueableMessage queueing_properties


