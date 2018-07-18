# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
bool load

# Sets the coefficient of friction that the controller will use for the contact point.
float64 coefficient_of_friction

# Sets the pose of the contact frame in the frame of the end effector body.
geometry_msgs/Pose body_frame_to_contact_frame

# Sets the contact normal used by the controller to load the contact point.
geometry_msgs/Vector3 contact_normal_in_world_frame


