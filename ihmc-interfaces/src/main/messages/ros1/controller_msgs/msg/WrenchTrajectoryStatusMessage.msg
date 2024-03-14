# This message is part of the IHMC whole-body controller API.
# The walking controller uses this message to report when it completed the execution of a trajectory message.

# The trajectory input was received and accepted, the execution just started.
int8 TRAJECTORY_EXECUTION_STATUS_STARTED=0

# The trajectory input was received, accepted, and its execution just finished.
int8 TRAJECTORY_EXECUTION_STATUS_COMPLETED=1

# Unique ID of the trajectory message this status refers to.
uint32 sequence_id

# Refers to the name of the end-effector that executed the trajectory.
string end_effector_name

# The current status for the trajectory execution.
# Field default value 255
int8 trajectory_execution_status

# Time in trajectory in seconds.
float64 timestamp


