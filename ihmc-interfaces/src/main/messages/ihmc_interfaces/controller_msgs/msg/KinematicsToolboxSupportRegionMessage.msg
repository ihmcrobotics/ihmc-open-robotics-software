# This message is part of the IHMC whole-body inverse kinematics module.
# The inverse kinematics solver provides the option to constrain the projection onto the horizontal plane of the center-of-mass
# to remain inside the given support polygon.
# This allows to constrain the solution to be statically stable.
# While the support polygon is usually determined based on the active controller, i.e. via CapturabilityBasedStatus or MultiContactBalanceStatus,
# this message can be used to directly specify it.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
float64 center_of_mass_margin -1
# The list of vertices of the support region.
# These are nominally identical to the robot's contact points, but might be different when a hand-hold is present, for example.
geometry_msgs/Point[] support_region_vertices

# The frame id of the vertex. If empty, world frame is assumed.
int64[] support_region_vertex_frames
