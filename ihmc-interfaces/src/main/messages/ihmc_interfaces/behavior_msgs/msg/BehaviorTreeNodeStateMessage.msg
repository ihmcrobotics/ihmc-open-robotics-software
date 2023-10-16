byte RUNNING = 0
byte FAILURE = 1
byte SUCCESS = 2
byte NOT_TICKED = 3

# The current status of the node
byte status

# The instant the node was last ticked
ihmc_common_msgs/InstantMessage last_tick_instant