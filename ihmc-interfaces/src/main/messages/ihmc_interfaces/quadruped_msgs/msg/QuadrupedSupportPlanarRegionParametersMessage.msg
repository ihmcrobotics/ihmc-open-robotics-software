# This message sets the parameters of the BipedalSupportPlanarRegionPublisher

# Enables the QuadrupedSupportPlanarRegionPublish module
bool enable false

# The support planar region is set to be a rectangle at each of the feet.

# This value specifies the distance from the foot to the inside portion of the vertices (under the robot).
float64 inside_support_region_size -1.0
# This value specifies the distance from the foot to the outside portion of the vertices (not under the robot).
float64 outside_support_region_size -1.0
