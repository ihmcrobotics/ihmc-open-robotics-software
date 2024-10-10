# This message is part of the IHMC whole-body controller API.
# Published by the IHMC controller, this message carries minimal information relative
# to the current balance status of the robot.
# All the information here is expressed in the world frame.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# This is the current value of omega, which is the time constant of the inverted pendulum.
float64 omega

# This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
geometry_msgs/Point capture_point_2d

# This is the desired position in world for the robot's capture point. Only x and y coordinates are relevant.
geometry_msgs/Point desired_capture_point_2d

# This is the measured position in world of the robot's center of mass.
geometry_msgs/Point center_of_mass_3d

# List of the active contact points used for the left foot. The coordinates are in world frame.
geometry_msgs/Point[] left_foot_support_polygon_3d

# List of the active contact points used for the right foot. The coordinates are in world frame.
geometry_msgs/Point[] right_foot_support_polygon_3d

# List of the active contact points for the left hand, empty if there is no hand contact point. The coordinates are in the hand body-fixed frame.
geometry_msgs/Point[] left_hand_contact_points

# List of the active contact points for the right hand, empty is there is no hand contact point. The coordinates are in the hand body-fixed frame.
geometry_msgs/Point[] right_hand_contact_points

# Surface normal of the left hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
geometry_msgs/Vector3 left_hand_contact_normal

# Surface normal of the right hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
geometry_msgs/Vector3 right_hand_contact_normal

# The estimated hand wrench of the right hand
controller_msgs/SpatialVectorMessage right_hand_wrench

# The estimated hand wrench of the left hand
controller_msgs/SpatialVectorMessage left_hand_wrench


