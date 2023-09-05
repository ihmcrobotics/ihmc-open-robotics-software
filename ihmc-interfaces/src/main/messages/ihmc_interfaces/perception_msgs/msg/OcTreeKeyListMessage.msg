# This message is part of the IHMC robot environment awareness module.
# This message contains a list of occupied cells in the octree
# All keys are given at maximum depth

int32 tree_depth
float64 tree_resolution
int32 number_of_keys

# See us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConvterter for info on the compression protocol
byte[<=200000] keys
