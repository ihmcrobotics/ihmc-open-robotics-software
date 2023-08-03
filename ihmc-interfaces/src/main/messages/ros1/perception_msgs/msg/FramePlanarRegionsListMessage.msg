# This message is part of the IHMC perception stack.
# This message contains a list of planar regions along with the sensor pose in world frame.
# A planar region is a finite area that lies on a 3D plane and that is delimited by a concave hull.
# As concave hulls are complex to manipulate, a set of convex polygons are also provided, altogether they approximate the area of the planar region.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Position of sensor when the planar region list was generated
geometry_msgs/Point sensor_position

# Orientation of sensor when the planar region list was generated
geometry_msgs/Quaternion sensor_orientation

# The PlanarRegionsList associated with the message.
perception_msgs/PlanarRegionsListMessage planar_regions


