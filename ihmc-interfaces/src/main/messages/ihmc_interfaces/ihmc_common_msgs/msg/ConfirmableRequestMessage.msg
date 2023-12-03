# A way of avoiding local data get overriden before
# it is received by peers. A message can be a request
# and confirmation at the same time.

# If this message is a request to change something.
bool is_request

# If this message is a confirmation that the request has been received by the other side.
bool is_confirmation

# Request number, monotonically increasing
uint32 request_number

# Confirmation number, the value of the last received request number
uint32 confirmation_number
