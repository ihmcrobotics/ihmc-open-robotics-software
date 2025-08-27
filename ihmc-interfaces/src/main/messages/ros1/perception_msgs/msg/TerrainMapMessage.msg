# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# X coordinate of the center of the terrain map
float64 map_center_x

# Y coordinate of the center of the terrain map
float64 map_center_y

# The number of cells per side of the terrain map grid
uint16 width_in_meters

# The number of cells contained per meter. This defines the resolution of the grid.
uint8 cells_per_meter

# Cell size in meters of an individual cell
float64 cell_size_in_meters

# The raw data for the terrain cost value, which are stored as chars.
int8[] terrain_cost_data

# The raw data for the contact map value, which are stored as chars.
int8[] contact_map_data

float32[] heights

# The raw data for the snap normal x value, which are stored as chars.
int8[] snapped_normal_x_data

# The raw data for the snap normal y value, which are stored as chars.
int8[] snapped_normal_y_data

# The raw data for the snap normal z value, which are stored as chars.
int8[] snapped_normal_z_data

# The raw data for the snapped support area, which are stored as chars.
int8[] snapped_area_data

# The raw data for the steppability area, which are stored as chars.
int8[] steppability_data

# The raw data for the steppable connections value, which are stored as chars.
int8[] steppable_connections_data


