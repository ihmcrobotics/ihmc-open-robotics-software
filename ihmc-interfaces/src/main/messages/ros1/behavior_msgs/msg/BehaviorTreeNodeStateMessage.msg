
# The node's unique ID
uint32 id

# A node is active if it lies on the path of the current tree tick.
bool is_active

# A mechanism for confirming and ending a freeze early
ihmc_common_msgs/ConfirmableRequestMessage confirmable_request


