# A way of avoiding local data get overriden before
# it is received by peers. A message can be a request
# and confirmation at the same time.

# Recent request numbers, monotonically increasing
# The size of this is roughly the max amount of expected dropped messages in 1 second
uint32[<=10] request_numbers

# Recent confirmation numbers, the values of received request numbers
# The size of this must be greater than or equal to the max timeout
uint32[<=250] confirmation_numbers
