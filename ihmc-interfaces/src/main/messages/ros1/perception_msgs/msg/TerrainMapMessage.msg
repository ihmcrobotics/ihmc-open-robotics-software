# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Height map message
perception_msgs/HeightMapMessage height_map

# Traversability score, on a 0 to 1 scale
float32[] traversability_score

# Classifying traversability according to SnapResult
int8[] traversability_class

# The raw data for the snap normal x value, which are stored as chars.
int8[] snapped_normal_x_data

# The raw data for the snap normal y value, which are stored as chars.
int8[] snapped_normal_y_data

# The raw data for the snap normal z value, which are stored as chars.
int8[] snapped_normal_z_data


