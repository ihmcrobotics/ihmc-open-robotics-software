# The base description of a behavior tree node.
# Note that descriptions have children however we can not put them
# in this message because ROS 2 messages cannot contain themselves.
# Instead we use a depth first ordered list and number of children
# to send trees over. This is contained in another higher level message.

# A mechanism for confirming and ending a freeze early
ihmc_common_msgs/ConfirmableRequestMessage confirmable_request

# The name of the node including .json if it's a JSON root node
string name

# Long form notes about the node.
# string notes

# Number of children
uint16 number_of_children
