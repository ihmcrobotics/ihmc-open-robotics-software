# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Discretization of the height map grid
float64 xy_resolution -1.0

# The height map covers a square of this width
float64 grid_size_xy -1.0

# X coordinate of the center of the height map
float64 grid_center_x 0.0

# Y coordinate of the center of the height map
float64 grid_center_y 0.0

# Z height offset for converting between floats and shorts
float64 height_offset 0.0

# Z height scale factor for converting between floats and shorts
float64 height_scale_factor 0.0

# Height of the ground plane, which is assumed to be flat
float64 estimated_ground_height 0.0

# Width of the height map in meters
float64 width_in_meters

# Cell size in meters of an individual cell
float64 cell_size_in_meters

# Cells per axis
int32 cells_per_axis

# List of height map keys. See HeightMapTools for converting keys to coordinates
int32[<=255000] keys

# List of heights, which correspond to the list of keys
int32[<=255000] heights

# List of variances, which correspond to the list of keys. May be empty.
float32[<=255000] variances

# List of centroids for each cell, which correspond to the list of keys. May be empty
# Note: The z coordinate of each point is ignored, but should correspond to the height.
geometry_msgs/Point[<=255000] centroids

# List of normals for each cell, which correspond to the list of keys. May be empty.
geometry_msgs/Vector3[<=255000] normals
