# Image message. It has a large capacity.
# It's recommended to reuse instances to avoid excessive memory allocation.

# Sequence number. Used for detecting out of order or dropped messages
uint32 sequence_number

# Use precise time measurements to measure delay between processes
uint32 acquisition_time_seconds_since_epoch
# Use precise time measurements to measure delay between processes
uint32 acquisition_time_additional_nanos

# Image width in pixels
uint16 image_width
# Image height in pixels
uint16 image_height

# The raw data for the image
byte[<=25000000] data

# The image format. Ordinal of OpenCVImageFormat
uint16 format

# Position of the focal point at sensor data aquisition
geometry_msgs/Point position
# Orientation of the focal point at sensor data aquisition
geometry_msgs/Quaternion orientation

# Intrinsic parameters for frustum camera models
IntrinsicParametersMessage intrinsic_parameters
