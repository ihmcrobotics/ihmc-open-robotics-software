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

# The desired position (in world coordinates) of the end-effector sent to the controller.
geometry_msgs/Point desired_end_effector_position

# The desired orientation (in world coordinates) of the end-effector sent to the controller.
geometry_msgs/Quaternion desired_end_effector_orientation

# The position (in world coordinates) of where the end-effector actually arrived when the trajectory finished.
geometry_msgs/Point actual_end_effector_position

# The orientation (in world coordinates) of where the end-effector actually arrived when the trajectory finished.
geometry_msgs/Quaternion actual_end_effector_orientation


