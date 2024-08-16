
# The instant the message was generated. Used also for equals comparison
ihmc_common_msgs/InstantMessage instant

# Log level - Debug(500), Info(400), Warn(300), or Error(200)
uint16 log_level

# The log message, as multiple ASCII bytes because of our 255 string length constraint
int8[] log_message


