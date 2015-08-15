## FootstepStatusMessage
# This message gives the status of the current footstep from the controller as well as the position
# and orientation of the footstep in world cooredinates. 

# Options for status
uint8 STARTED=0 # execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state
uint8 COMPLETED=1 # a footstep is completed
uint8 status

# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given
# FootstepDataListMessage.
int32 footstep_index

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

# actualFootPositionInWorld gives the position of where the foot actually landed as opposed
# to the desired position sent to the controller
geometry_msgs/Vector3 actual_foot_position_in_world

# actualFootOrientationInWorld gives the orientation the foot is actually in as opposed tothe desired orientation sent to the controller
geometry_msgs/Quaternion actual_foot_orientation_in_world

# isDoneWalking will be set to true when in double-support and there are no footsteps queued. If
# isDoneWalking is set to true, the rest of the fields in this packet should be ignored
bool is_done_walking


