## FootstepStatusMessage
# This message gives the status of the current footstep from the controller.

# Options for status
# STARTED = 0 - indicates that the execution of a footstep has begun
# COMPLETED = 1 - a footstep is completed
uint8 status

# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given
# FootstepDataListMessage.
int32 footstep_index

# Options for robotSide
# LEFT = 0 - refers to the LEFT side of a robot
# RIGHT = 1 - refers to the RIGHT side of a robot
uint8 robot_side

geometry_msgs/Vector3 actual_foot_position_in_world

geometry_msgs/Quaternion actual_foot_orientation_in_world

bool is_done_walking


