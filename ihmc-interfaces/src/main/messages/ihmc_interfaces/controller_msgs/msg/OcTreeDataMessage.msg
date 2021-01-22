# This message is part of the IHMC robot environment awareness module.
# This message contains a list of occupied cells in the octree
# All keys are given at maximum depth

int32 maximum_tree_depth
float64 tree_resolution

int32[<=5000] x_keys
int32[<=5000] y_keys
int32[<=5000] z_keys
int32[<=5000] tree_depth
