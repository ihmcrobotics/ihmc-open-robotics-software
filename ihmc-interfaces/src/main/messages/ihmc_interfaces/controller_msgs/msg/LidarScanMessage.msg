# This message should disappear to be replaced by ROS equivalent.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int64 robot_timestamp
geometry_msgs/Point lidar_position
geometry_msgs/Quaternion lidar_orientation
float32[] scan
