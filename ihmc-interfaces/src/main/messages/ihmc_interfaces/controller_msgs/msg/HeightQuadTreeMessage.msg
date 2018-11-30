# This message is part of the IHMC height quad tree module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float32 default_height
float32 resolution
float32 size_x
float32 size_y
HeightQuadTreeLeafMessage[<=5000] leaves