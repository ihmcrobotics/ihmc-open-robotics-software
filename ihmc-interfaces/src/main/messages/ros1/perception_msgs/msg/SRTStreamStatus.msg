
# Connection information
string streamer_address

uint16 streamer_port

bool is_streaming

float32 expected_publish_frequency

# Camera intrinsics
uint16 image_width

uint16 image_height

float32 fx

float32 fy

float32 cx

float32 cy

float32 depth_discretization

# Camera model of the sensor. Ordinal of us.ihmc.perception.CameraModel
uint8 camera_model

# Some codecs cannot accept extra data, so it's sent through this message instead
perception_msgs/VideoFrameExtraData frame_extra_data

bool contains_extra_data


