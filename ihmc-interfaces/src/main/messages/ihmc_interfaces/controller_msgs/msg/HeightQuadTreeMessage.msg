# This message is part of the IHMC height quad tree module

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

float32 default_height
float32 resolution
float32 size_x
float32 size_y
HeightQuadTreeLeafMessage[<=5000] leaves