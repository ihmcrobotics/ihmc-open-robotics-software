# This message is part of the IHMC whole-body controller API.
# The walking controller uses this message to report when it completed the execution of a trajectory message.

byte TRAJECTORY_EXECUTION_STATUS_STARTED = 0
byte TRAJECTORY_EXECUTION_STATUS_COMPLETED = 1

# Unique ID of the trajectory message this status refers to.
uint32 sequence_id

# Refers to the names of the joints that executed the trajectory.
string[] joint_names

# The current status for the trajectory execution.
byte trajectory_status 255

# The desired positions of the joints as sent to the controller.
float64[] desired_joint_positions
# The positions of where the joints actually arrived when the trajectory finished.
float64[] actual_joint_positions
