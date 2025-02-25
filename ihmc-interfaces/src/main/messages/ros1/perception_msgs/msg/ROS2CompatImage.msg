# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
# Header frame_id should be optical frame of camera
# origin of frame should be optical center of cameara
# +x should point to the right in the image
# +y should point down in the image
# +z should point into to plane of the image
# If the frame_id here and the frame_id of the CameraInfo
# message associated with the image conflict
# the behavior is undefined
# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.
# taken from the list of strings in include/sensor_msgs/image_encodings.hpp

# Header timestamp should be acquisition time of image
Header header

# image height, that is, number of rows
uint32 height

# image width, that is, number of columns
uint32 width

# Encoding of pixels -- channel meaning, ordering, size
string encoding

# is this data bigendian?
uint8 is_bigendian

# Full row length in bytes
uint32 step

# actual matrix data, size is (step * rows)
uint8[] data


