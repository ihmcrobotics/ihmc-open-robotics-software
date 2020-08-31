# This message is part of the IHMC whole-body inverse kinematics module.
# It holds all the information needed for specifying the contact state of the robot.
# The inverse kinematics solver provides the option to constrain the projection onto the horizontal plane of the center-of-mass
# to remain inside the active support polygon also projected onto the horizontal plane.
# This allows to constrain the solution to be statically stable.
# While the support polygon is usually determined based on the active controller, i.e. via CapturabilityBasedStatus or MultiContactBalanceStatus,
# this message can be used to directly specify it.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
float64 center_of_mass_margin -1
# The list of active contact points to use evaluate the support polygon.
# Each contact point is expected to be expressed in body frame.
geometry_msgs/Point[<=20] contact_points_in_body_frame
# The id used to retrieve the contacting body for each contact point.
int32[<=20] contacting_body_ids
