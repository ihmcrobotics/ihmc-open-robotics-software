# This message is part of the IHMC walking controller preview module: WalkingControllerPreviewToolbox.
# Configure only the fields to be previewed.
# New fields will be added to this message as the module supports them.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Set this field to request a preview of the corresponding walking sequence.
controller_msgs/FootstepDataListMessage footsteps