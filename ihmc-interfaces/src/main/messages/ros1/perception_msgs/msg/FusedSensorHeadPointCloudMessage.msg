# Message for sending large colored point clouds,
# discretized and compressed with LZ4,
# and publishing partial scans at a higher rate to allow smaller message sizes

# Approximate sensor data aquisition time seconds since epoch
# Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
int64 aquisition_seconds_since_epoch

# Approximate sensor data aquisition time additional nanoseconds
int64 aquisition_additional_nanos

# Total number of segments
uint32 number_of_segments

# Number of points per segment
int32 points_per_segment

# Segment this data represents
uint32 segment_index

# Compressed point cloud data
# Sized specifically for our current primary sensor head streaming
# 22220 * 4 * 4
int8[] scan


