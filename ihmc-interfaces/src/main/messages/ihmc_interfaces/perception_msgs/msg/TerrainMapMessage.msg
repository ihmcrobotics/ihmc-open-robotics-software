# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id


# The number of cells per side of the terrain map grid
uint16 local_grid_size
# The number of cells contained per meter. This defines the resolution of the grid.
uint8 cells_per_meter

# X coordinate of the center of the terrain map
float64 map_center_x

# Y coordinate of the center of the terrain map
float64 map_center_y


# These are the scale factors that are used to convert from the short values to the actual heights.
# First the shorts are masked with 0xFFFF. Then they are scaled down by the scale factor and offset by the offset.
# height = short & 0xFFFF / height_scale_factor - height_scale_offset
float64 height_scale_factor
float64 height_scale_offset

# The raw data for the raw heights, which are stored as shorts.
byte[<=500000] height_map_data
# The raw data for the snapped heights, which are stored as shorts.
byte[<=500000] snapped_height_data
# The raw data for the snap normal x value, which are stored as chars.
byte[<=250000] snapped_normal_x_data
# The raw data for the snap normal y value, which are stored as chars.
byte[<=250000] snapped_normal_y_data
# The raw data for the snap normal z value, which are stored as chars.
byte[<=250000] snapped_normal_z_data
# The raw data for the snapped support area, which are stored as chars.
byte[<=250000] snapped_area_data
