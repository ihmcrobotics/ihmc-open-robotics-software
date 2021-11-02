# Which segment
uint32 segment_index

# Compressed point cloud data
# Sized specifically for our current primary sensor head streaming
# 22220 * 4 * 4
byte[<=355520] scan
