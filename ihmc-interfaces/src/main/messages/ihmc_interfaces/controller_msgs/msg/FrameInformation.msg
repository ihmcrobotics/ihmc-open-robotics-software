# This message is part of the IHMC whole-body controller API.
# This message carries the frame related information needed for some messages such as taskspace trajectories.
# Valid codes and their associated frames include:"
int64 WORLD_FRAME = 83766130
int64 MIDFEET_ZUP_FRAME = -100
int64 PELVIS_ZUP_FRAME = -101
int64 PELVIS_FRAME = -102
int64 CHEST_FRAME = -103
int64 CENTER_OF_MASS_FRAME = -104
int64 LEFT_SOLE_FRAME = -105
int64 RIGHT_SOLE_FRAME = -106

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# The ID of the reference frame that a trajectory is executed in.
int64 trajectory_reference_frame_id
# The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the
# trajectory data will be switched to the trajectory frame immediately when the message is received
# by the controller. If set to the value 0 it will be
# assumed that this is the same frame as the trajectory frame.
# It is recommended that this should be the same frame as the trajectory_reference_frame_id to
# avoid unexpected behavior. When different, the controller will change the data to be expressed
# in the trajectory frame at reception of the message.
# The data frame is only useful if the user is unable to change the frame the data is expressed in
# to the trajectory frame. However, unexpected behavior might occur if the data frame is moving
# with respect to the trajectory frame during execution. To highlight this consider the following
# example:
# A hand trajectory needs to be executed while the robot walks to a location in world. The hand
# trajectory might be known in world frame but for safety the trajectory execution frame is set
# to a frame attached to the robot. If the data is packed in world and the data frame is set to world
# this will cause the resulting trajectory to be wrong since the transformation to trajectory frame
# happens at the start of execution rather than every controller tick.
int64 data_reference_frame_id 0