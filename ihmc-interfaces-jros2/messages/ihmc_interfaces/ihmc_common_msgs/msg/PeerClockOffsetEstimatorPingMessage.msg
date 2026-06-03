# This message is a request-reply ping used to estimate the clock offset
# of another node to be used by data synchronization algorithms.

# If this is a request message, else it's a reply message
bool is_request

# The guid of the publisher that's expected to send back a reply
ihmc_common_msgs/GuidMessage request_target

# The guid of the publisher associated with the requester
ihmc_common_msgs/GuidMessage reply_target

# The time at which the request is sent
ihmc_common_msgs/InstantMessage request_send_time

# The time at which the reply is sent
ihmc_common_msgs/InstantMessage reply_send_time
