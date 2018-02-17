## ArmTrajectoryRosMessage
# This message commands the controller to move an arm in jointspace to the desired joint angles while
# going through the specified trajectory points. A third order polynomial function is used to
# interpolate between trajectory points. The jointTrajectoryMessages can have different waypoint times
# and different number of waypoints. If a joint trajectory message is empty, the controller will hold
# the last desired joint position while executing the other joint trajectories. A message with a
# unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
# This rule does not apply to the fields of this message.

# Specifies the side of the robot that will execute the trajectory.
int8 robot_side

# Trajectories for each joint.
ihmc_msgs/JointspaceTrajectoryRosMessage jointspace_trajectory

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


