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

bool has_terrain_cost_data false
bool has_contact_map_data false
bool has_height_map_float_data false
bool has_snapped_normal_data false
bool has_snapped_area_data false
bool has_steppability_data false
bool has_steppable_connections_data false

# The raw data for the terrain cost value, which are stored as chars.
byte[<=250000] terrain_cost_data
# The raw data for the contact map value, which are stored as chars.
byte[<=250000] contact_map_data
float32[<=255000] heights
# The raw data for the snap normal x value, which are stored as chars.
byte[<=250000] snapped_normal_x_data
# The raw data for the snap normal y value, which are stored as chars.
byte[<=250000] snapped_normal_y_data
# The raw data for the snap normal z value, which are stored as chars.
byte[<=250000] snapped_normal_z_data
# The raw data for the snapped support area, which are stored as chars.
byte[<=250000] snapped_area_data
# The raw data for the steppability area, which are stored as chars.
byte[<=250000] steppability_data
# The raw data for the steppable connections value, which are stored as chars.
byte[<=250000] steppable_connections_data