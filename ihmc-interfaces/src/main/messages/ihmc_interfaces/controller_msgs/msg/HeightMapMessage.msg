# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 xy_resolution -1.0
float64 grid_size_xy -1.0

int32[<=30000] x_cells
int32[<=30000] y_cells
float32[<=30000] heights
