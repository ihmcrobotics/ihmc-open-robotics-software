# Image message. It has a large capacity.
# It's recommended to reuse instances to avoid excessive memory allocation.

# Sequence number. Used for detecting out of order or dropped messages
uint32 sequence_number

# The instant right after we got data from the sensor. Used to measure delay between processes
ihmc_common_msgs/InstantMessage acquisition_time

# Image width in pixels
uint16 image_width

# Image height in pixels
uint16 image_height

# The raw data for the image
int8[] data

# The image format. Ordinal of OpenCVImageFormat
uint16 format

# Position of the focal point at sensor data aquisition
geometry_msgs/Point position

# Orientation of the focal point at sensor data aquisition
geometry_msgs/Quaternion orientation

# Intrinsic parameters for frustum camera models
perception_msgs/IntrinsicParametersMessage intrinsic_parameters


