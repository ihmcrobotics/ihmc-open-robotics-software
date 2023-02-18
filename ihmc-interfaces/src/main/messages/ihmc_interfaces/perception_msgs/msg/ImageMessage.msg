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
byte[<=25000000] data

# The image format. Ordinal of OpenCVImageFormat
uint16 format

# Position of the focal point at sensor data aquisition
geometry_msgs/Point position
# Orientation of the focal point at sensor data aquisition
geometry_msgs/Quaternion orientation

# Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model
bool is_pinhole_camera_model

# Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
bool is_equidistant_fisheye_camera_model

# Is Ouster camera model
bool is_ouster_camera_model

# Horizontal focal length in units of pixels (Fx)
float32 focal_length_x_pixels

# Vertical focal length in units of pixels (Fx)
float32 focal_length_y_pixels

# Principal point X in units of pixels (Cx)
float32 principal_point_x_pixels

# Principal point Y in units of pixels (Cy)
float32 principal_point_y_pixels

# Depth discretization, if a depth image. Depth images are sent as unsigned 16 bit integers
# in units of this value
float32 depth_discretization

# If Ouster camera model, the vertical field of view in radians
float32 ouster_vertical_field_of_view

# If Ouster camera model, the horizontal field of view in radians
float32 ouster_horizontal_field_of_view
