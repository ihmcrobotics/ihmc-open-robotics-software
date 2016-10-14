## OneDoFJointTrajectoryRosMessage
# This class is used to build trajectory messages in jointspace. It holds all the trajectory points to
# go through with a one-dimensional trajectory. A third order polynomial function is used to
# interpolate between trajectory points.

# List of trajectory points to go through while executing the trajectory.
ihmc_msgs/TrajectoryPoint1DRosMessage[] trajectory_points

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


