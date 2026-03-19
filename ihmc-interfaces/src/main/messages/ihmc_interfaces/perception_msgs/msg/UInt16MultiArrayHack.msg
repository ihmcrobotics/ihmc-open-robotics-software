# This is a stupid copy of the UInt16MultiArray message to get enough array elements
# because ihmc-ros2-library defaults unbounded arrays to 100 elements.
# TODO: jros2 Remove this message when we transition to jros2

std_msgs/MultiArrayLayout   layout        # specification of data layout
uint16[<=512]               data          # array of data