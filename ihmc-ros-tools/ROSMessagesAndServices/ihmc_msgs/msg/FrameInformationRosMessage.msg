## FrameInformationRosMessage
# This is a holder for frame related information. Valid codes and their associated frames include:
# MIDFEET_ZUP_FRAME = -100 PELVIS_ZUP_FRAME = -101 PELVIS_FRAME = -102 CHEST_FRAME = -103
# CENTER_OF_MASS_FRAME = -104 LEFT_SOLE_FRAME = -105 RIGHT_SOLE_FRAME = -106

# The ID of the reference frame that a trajectory is executed in.
int64 trajectory_reference_frame_id

# The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the
# trajectory data will be switched to the trajectory frame immediately when the message is received by
# the controller.
int64 data_reference_frame_id

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


