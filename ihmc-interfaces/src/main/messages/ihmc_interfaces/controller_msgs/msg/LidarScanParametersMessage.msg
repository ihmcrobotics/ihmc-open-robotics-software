# Message used for simulated LIDAR

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int64 timestamp

float32 sweep_yaw_max
float32 sweep_yaw_min
float32 height_pitch_max
float32 height_pitch_min
float32 time_increment
float32 scan_time
float32 min_range
float32 max_range

int32 points_per_sweep
int32 scan_height