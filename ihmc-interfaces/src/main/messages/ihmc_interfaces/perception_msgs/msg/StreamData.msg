# THIS MESSAGE IS SENT OVER VIDEO STREAMS WITH SRT.
# MAX SIZE IS 881 BYTES. DO NO EXCEED THAT SIZE

# Camera intrinsics
uint16 image_width
uint16 image_height
float32 fx
float32 fy
float32 cx
float32 cy
float32 depth_discretization

# Frame acquisition time
ihmc_common_msgs/InstantMessage acquisition_time

# Sensor pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
