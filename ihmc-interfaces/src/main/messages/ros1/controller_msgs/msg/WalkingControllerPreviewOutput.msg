# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Interval in time between two frames.
float64 frame_dt

# List of configurations for each key frames.
controller_msgs/KinematicsToolboxOutputStatus[] robot_configurations


