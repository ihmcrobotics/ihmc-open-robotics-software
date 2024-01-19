# A way of avoiding local data get overriden before
# it is received by peers. A message can be a request
# and confirmation at the same time.

# Recent request numbers, monotonically increasing
uint32[] request_numbers

# Recent confirmation numbers, the values of received request numbers
uint32[] confirmation_numbers


