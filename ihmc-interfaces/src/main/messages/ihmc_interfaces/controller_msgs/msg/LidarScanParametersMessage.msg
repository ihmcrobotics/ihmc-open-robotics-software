# Message used for simulated LIDAR

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

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