# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# X coordinate of the center of the height map
float64 grid_center_x 0.0

# Y coordinate of the center of the height map
float64 grid_center_y 0.0

# Width of the height map in meters
float64 width_in_meters

# Cell size in meters of an individual cell
float64 cell_size_in_meters

# Cells per axis
int32 cells_per_axis

# Height map in metric values
float32[<=255000] height_map

# Obstacle clearance score, on a 0 to 1 scale
float32[<=255000] obstacle_clearance_score
# Traversability score, on a 0 to 1 scale
float32[<=255000] traversability_score
# Classifying traversability according to SnapResult
byte[<=255000] traversability_class

# The raw data for the snap normal x value, which are stored as chars.
byte[<=255000] snapped_normal_x_data
# The raw data for the snap normal y value, which are stored as chars.
byte[<=255000] snapped_normal_y_data
# The raw data for the snap normal z value, which are stored as chars.
byte[<=255000] snapped_normal_z_data
