## SupportPolygonRosMessage
# This message contains of an array of points that represent the convex hull of a support polygon for
# a single robot foot
# Constant defining max number of possible elements in the array of points
int32 MAXIMUM_NUMBER_OF_VERTICES=8

# The number of vertices in the array of points
int32 number_of_vertices

# The vertices of the support polygon
ihmc_msgs/Point2dRosMessage[] vertices

