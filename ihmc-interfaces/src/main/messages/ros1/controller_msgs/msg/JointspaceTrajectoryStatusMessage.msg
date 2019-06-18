# The trajectory input was received and accepted, the execution just started.
int8 TRAJECTORY_EXECUTION_STATUS_STARTED=0

# The trajectory input was received, accepted, and its execution just finished.
int8 TRAJECTORY_EXECUTION_STATUS_COMPLETED=1

# Unique ID of the trajectory message this status refers to.
uint32 sequence_id

# Refers to the names of the joints that executed the trajectory.
string[] joint_names

# The current status for the trajectory execution.
# Field default value 255
int8 trajectory_execution_status

# Time in trajectory in seconds.
float64 timestamp

# The desired positions of the joints as sent to the controller.
float64[] desired_joint_positions

# The positions of where the joints actually arrived when the trajectory finished.
float64[] actual_joint_positions


