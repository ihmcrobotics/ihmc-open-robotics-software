# This message is part of the IHMC multi-contact controller API.
# Published by the IHMC controller, this message carries minimal information relative
# to the balance status of the robot.
# All the information here is expressed in the world frame.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
geometry_msgs/Point capture_point_2d

# This is the measured position in world of the robot's center of mass.
geometry_msgs/Point center_of_mass_3d

# List of the active contact points. The coordinates are in world frame. Only x and y coordinates are relevant.
geometry_msgs/Point[] support_polygon

# List of the rigid-bodies in contact. This list's size and ordering corresponds to the support_polygon list.
int32[] support_rigid_body_ids

# First part of the estimation of the local ground plane: the origin in world coordinates of the ground plane.
geometry_msgs/Point ground_plane_estimate_origin

# Second part of the estimation of the local ground plane: the normal in world coordinates of the ground plane.
geometry_msgs/Vector3 ground_plane_estimate_normal


