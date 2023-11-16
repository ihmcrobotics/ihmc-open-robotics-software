# The base description of a behavior tree node.
# Note that descriptions have children however we can not put them
# in this message because ROS 2 messages cannot contain themselves.
# Instead we use a depth first ordered list and number of children
# to send trees over. This is contained in another higher level message.

# A human readable description of what the node does
string description

# Number of children
uint16 number_of_children

# JSON file name if this node is the root of a JSON file
string json_file_name


