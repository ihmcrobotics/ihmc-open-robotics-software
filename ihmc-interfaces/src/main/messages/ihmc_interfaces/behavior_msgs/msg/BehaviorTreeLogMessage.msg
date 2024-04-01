# The instant the message was generated. Used also for equals comparison
ihmc_common_msgs/InstantMessage instant

# Log level - Debug(500), Info(400), Warn(300), or Error(200)
uint16 log_level

# The ID of the node that generated the message
uint16 source_node_id

# The log message
string message
