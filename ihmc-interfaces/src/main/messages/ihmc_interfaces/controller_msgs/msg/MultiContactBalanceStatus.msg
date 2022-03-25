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
# List of the active contact points expressed in world frame. Only x and y coordinates are relevant.
geometry_msgs/Point[<=16] support_polygon
# List of the active contact points expressed in local body-fixed frame.
geometry_msgs/Point[<=16] contact_points_in_body
# List of the rigid-bodies in contact. This list's size and ordering corresponds to the support_polygon and contact_point_in_body lists.
int32[<=16] support_rigid_body_ids
# (Optional) List of the contact surface normals in world frame. If provided these are used to solve generalized support region.
geometry_msgs/Vector3[<=16] surface_normals_in_world
