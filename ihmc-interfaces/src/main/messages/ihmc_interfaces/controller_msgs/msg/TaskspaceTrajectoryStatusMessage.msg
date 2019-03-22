# This message is part of the IHMC whole-body controller API.
# The walking controller uses this message to report when it completed the execution of a trajectory message.

byte TRAJECTORY_EXECUTION_STATUS_STARTED = 0
byte TRAJECTORY_EXECUTION_STATUS_COMPLETED = 1

# Unique ID of the trajectory message this status refers to.
uint32 sequence_id

# Refers to the name of the rigid-body that executed the trajectory.
string body_name

# The current status for the trajectory execution.
byte trajectory_status 255

# The desired position (in world coordinates) of the rigid-body sent to the controller.
geometry_msgs/Point desired_body_position
# The desired orientation (in world coordinates) of the rigid-body sent to the controller.
geometry_msgs/Quaternion desired_foot_orientation
# The position (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
geometry_msgs/Point actual_body_position
# The orientation (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
geometry_msgs/Quaternion actual_body_orientation