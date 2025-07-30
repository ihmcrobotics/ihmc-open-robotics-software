# This is the location in world of the center of the chunk
float64 center_x
float64 center_y

# We have a hash to keep each chunk unique
int32 hash_code_of_chunk

# How we store the data for the chunk
perception_msgs/HeightMapMessage chunk
