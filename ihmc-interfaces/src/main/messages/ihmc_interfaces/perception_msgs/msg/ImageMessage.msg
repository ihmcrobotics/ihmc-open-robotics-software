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

# Depth discretization unit length.
# This is used when this image is a depth image. The image will be represented
# in data as unsigned integers where the units are this distance.
# For instance a depth pixel value of 2 and discretization value of 0.1 would
# indicate a depth of 0.2 meters.
float32 depth_discretization

# The raw data for the image
byte[<=25000000] data

# The image format. Ordinal of OpenCVImageFormat
uint8 format

# Position of the focal point at sensor data acquisition
geometry_msgs/Point position

# Orientation of the focal point at sensor data acquisition
geometry_msgs/Quaternion orientation

# Camera model of the sensor. Ordinal of us.ihmc.perception.CameraModel
uint8 camera_model

# Horizontal focal length in units of pixels (Fx)
float32 focal_length_x_pixels

# Vertical focal length in units of pixels (Fx)
float32 focal_length_y_pixels

# Principal point X in units of pixels (Cx)
float32 principal_point_x_pixels

# Principal point Y in units of pixels (Cy)
float32 principal_point_y_pixels

# If Ouster camera model, the calibrated beam altitude angles used to get 3D points.
float32[<=128] ouster_beam_altitude_angles

# If Ouster camera model, the calibrated beam azimuth angles used to get 3D points.
float32[<=128] ouster_beam_azimuth_angles
