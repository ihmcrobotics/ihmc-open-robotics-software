# This message is part of the IHMC whole-body controller API.
# Message carrying the information needed to enable load bearing for a end-effector.
# A contact point will be used to enable this feature. This point is attached to the end-effector.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
bool load false
# Sets the coefficient of friction that the controller will use for the contact point.
float64 coefficient_of_friction 0
# Pose of the contact frame, expressed in the frame of the end effector body.
# The contact frame origin is the contact point and the z-axis of contact frame is the contact normal (points into the environment)
geometry_msgs/Pose contact_pose_in_body_frame