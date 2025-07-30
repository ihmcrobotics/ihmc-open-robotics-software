# Most of the data we want is actually stored inside the HeightMapMessage, so we don't have to worry about storing it here, we just need a hash

# We have a hash to keep each chunk unique
int32 hash_code_of_chunk

# How we store the data for the chunk
perception_msgs/HeightMapMessage chunk
