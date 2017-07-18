## FrameInformationRosMessage
# This is a holder for frame related information

# The ID of the reference frame that a trajectory is executed in.
int64 trajectory_reference_frame_id

# The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the
# trajectory data will be switched to the trajectory frame immediately when the message is received by
# the controller.
int64 data_reference_frame_id


