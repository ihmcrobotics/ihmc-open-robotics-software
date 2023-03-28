# This message sets the parameters of the BipedalSupportPlanarRegionPublisher
# The support planar region is set to be a rectangle at each of the feet.

# Enables the QuadrupedSupportPlanarRegionPublish module
bool enable

# This value specifies the distance from the foot to the inside portion of the vertices (under the robot).
# Field default value -1.0
float64 inside_support_region_size

# This value specifies the distance from the foot to the outside portion of the vertices (not under the robot).
# Field default value -1.0
float64 outside_support_region_size


