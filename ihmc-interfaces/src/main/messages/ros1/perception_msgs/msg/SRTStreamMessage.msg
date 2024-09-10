# #############################
# ## REQUEST MESSAGE FIELDS ###
# #############################
# #########################
# ## ACK MESSAGE FIELDS ###
# #########################
# ############################
# ## Status message fields ###
# ############################

# UUID of message
# Request: set to random UUID
# Reply: match the request UUID
# Status: 0
ihmc_common_msgs/UUIDMessage id

# Address to send the stream to
string receiver_address

# Port to send the stream to
uint16 receiver_port

# Whether to connect or disconnect
bool connection_wanted

# Camera intrinsics
uint16 image_width

uint16 image_height

float32 fx

float32 fy

float32 cx

float32 cy

float32 depth_discretization

# Position of the focal point at sensor data acquisition
geometry_msgs/Point position

# Orientation of the focal point at sensor data acquisition
geometry_msgs/Quaternion orientation


