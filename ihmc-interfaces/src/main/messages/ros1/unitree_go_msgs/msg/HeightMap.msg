
# Header
# timestamp
float64 stamp

# world frame id
string frame_id

# Map info
# The map resolution [m/cell]
float32 resolution

# Map width along x-axis [cells]
uint32 width

# Map height alonge y-axis [cells]
uint32 height

# Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame
float32[] origin

# Map data, in x-major order, starting with [0,0], ending with [width, height]
# For a cell whose 2d-array-index is [ix, iy]，
# its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]
# its cell value is: data[width * iy + ix]
float32[] data


