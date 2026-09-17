# This is a stupid copy of the Float32MultiArray message to get enough array elements
# because ihmc-ros2-library defaults unbounded arrays to 100 elements.
# Large-capacity Float32MultiArray (1024 elements) for mask polygon data.

std_msgs/MultiArrayLayout   layout        # specification of data layout
float32[<=1024]             data          # array of data
