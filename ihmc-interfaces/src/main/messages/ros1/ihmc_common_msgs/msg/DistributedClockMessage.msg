# This message is a request-reply ping used to estimate the clock offset
# of another node to be used by data synchronization algorithms.

# The requester's unique ID
string requester_id

# The replier's unique ID
string replier_id

# Monotonically increasing request number
uint64 sequence_id

# The time at which the request is sent
ihmc_common_msgs/InstantMessage request_send_time

# The time at which the reply is sent
ihmc_common_msgs/InstantMessage reply_send_time


