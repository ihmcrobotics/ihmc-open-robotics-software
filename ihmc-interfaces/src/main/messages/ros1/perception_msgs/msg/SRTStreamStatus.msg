
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

# Sensor pose
geometry_msgs/Point position

geometry_msgs/Quaternion orientation


