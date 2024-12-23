# The base description of a behavior tree node.
# Note that descriptions have children however we can not put them
# in this message because ROS 2 messages cannot contain themselves.
# Instead we use a depth first ordered list and number of children
# to send trees over. This is contained in another higher level message.
# Long form notes about the node.
# string notes

# The timestamp and modifer ID of the latest modification to this node's children set
ihmc_common_msgs/LatestModificationMessage latest_modification_to_children

# The timestamp and modifer ID of the latest modification to this node's data fields
ihmc_common_msgs/LatestModificationMessage latest_modification_to_data

# The name of the node including .json if it's a JSON root node
string name

# Number of children
uint16 number_of_children


