# We have a hash to keep each chunk unique
int32 hash_code_of_chunk

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# X coordinate of the center of the height map
float32 origin_x 0.0

# Y coordinate of the center of the height map
float32 origin_y 0.0

# Width of the height map in meters
float32 width_in_meters

# Cell size in meters of an individual cell
float32 cell_size_in_meters

# Cells per axis
int32 cells_per_axis

# List of heights, which correspond to the list of keys
byte[<=20000] heights
