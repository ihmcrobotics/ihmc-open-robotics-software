# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1.0
float64 xy_resolution

# Field default value -1.0
float64 grid_size_xy

float64 grid_center_x

float64 grid_center_y

float64 estimated_ground_height

int32[] cells

float32[] heights


