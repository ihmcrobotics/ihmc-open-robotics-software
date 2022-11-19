# This message is part of the ETHZ's GPU Height Map Module.
# This message contains GPU GridMap Message.
# std_msgs/Header header
# uint32 seq
# time stamp
# string frame_id
# float64 resolution
# float64 length_x
# float64 length_y
# geometry_msgs/Pose pose
# # geometry_msgs/Point position
# ## float64 x
# ## float64 y
# ## float64 z
# # geometry_msgs/Quaternion orientation
# ## float64 x
# ## float64 y
# ## float64 z
# ## float64 w
# std_msgs/MultiArrayLayout layout
# # std_msgs/MultiArrayDimension[] dim
# ## string label
# ## uint32 size
# ## uint32 stride
# # uint32 data_offset
# float32[] data

grid_map_msgs/GridMapInfo info

string[] layers

string[] basic_layers

std_msgs/Float32MultiArray[] data

uint16 outer_start_index

uint16 inner_start_index


