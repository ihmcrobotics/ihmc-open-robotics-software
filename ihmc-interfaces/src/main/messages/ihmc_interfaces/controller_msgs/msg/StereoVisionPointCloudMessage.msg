# Should disappear for the ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 robot_timestamp
float32[<=600000] point_cloud
int32[<=200000] colors