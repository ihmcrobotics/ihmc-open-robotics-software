## FootstepStatusMessage
# This message gives the status of the current footstep from the controller.

#Options for status
# uint8 STARTED = 0
# uint8 COMPLETED = 1
uint8 status

# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given
# FootstepDataListMessage.
int32 footstep_index

int32 robot_side

geometry_msgs/Vector3 actual_foot_position_in_world

geometry_msgs/Quaternion actual_foot_orientation_in_world

bool is_done_walking


