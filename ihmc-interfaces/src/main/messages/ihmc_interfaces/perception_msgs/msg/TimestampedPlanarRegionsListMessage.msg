# This message contains a list of planar regions and a timestamp.

# Approximate last update time seconds since epoch
# Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
int64 last_updated_seconds_since_epoch
# Approximate last update time additional nanoseconds
int64 last_updated_additional_nanos

perception_msgs/PlanarRegionsListMessage planar_regions