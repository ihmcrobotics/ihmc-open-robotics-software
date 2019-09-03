# This message can be used as a general image.
# Each component of the data which of length is same with width * height is an integer value describing color with the conventional RGB.
# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 width

int32 height

int32[<=4000000] rgbdata