# This is a stupid copy of the UInt16MultiArray message to get enough array elements
# because ihmc-ros2-library defaults unbounded arrays to 100 elements.
# Large-capacity UInt16MultiArray for perception data.

std_msgs/MultiArrayLayout   layout        # specification of data layout
uint16[<=1024]               data          # array of data