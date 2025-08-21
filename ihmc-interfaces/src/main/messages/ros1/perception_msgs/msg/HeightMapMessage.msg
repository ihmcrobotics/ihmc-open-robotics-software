# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# X coordinate of the center of the height map
float64 grid_center_x

# Y coordinate of the center of the height map
float64 grid_center_y

# Z height offset for converting between floats and shorts
float64 height_offset

# Z height scale factor for converting between floats and shorts
float64 height_scale_factor

# Width of the height map in meters
float64 width_in_meters

# Cell size in meters of an individual cell
float64 cell_size_in_meters

# Cells per axis
int32 cells_per_axis

# List of heights, which correspond to the list of keys
float32[] heights


