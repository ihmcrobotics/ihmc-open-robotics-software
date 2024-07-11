# The base description of a behavior tree node.
# Note that descriptions have children however we can not put them
# in this message because ROS 2 messages cannot contain themselves.
# Instead we use a depth first ordered list and number of children
# to send trees over. This is contained in another higher level message.
# Long form notes about the node.
# string notes

# The name of the node including .json if it's a JSON root node
string name

# Number of children
uint16 number_of_children


