# This message is part of the IHMC height map module
# A crop of the height map around the robot for the controller. Each cell is a little-endian int16, where
# height = height_offset + height_resolution * value and NO_DATA_VALUE means unknown. The cells are deflate-compressed
# so a map fits in a few IP fragments: one lost fragment drops the whole best-effort sample.

int16 NO_DATA_VALUE=-32768

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

# Height in meters of a cell with value 0
float32 height_offset

# Meters per quantization step
float32 height_resolution

# Deflate-compressed int16 cells, ordered by key
byte[] compressed_heights
