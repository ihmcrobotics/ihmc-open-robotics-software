# A way of avoiding local data get overriden before
# it is received by peers.

# Nothing special has happened. This should trigger no effects.
int8 NOOP=0

# This message is a request to change something.
int8 REQUEST=1

# Confirmation that the request has been received by the other side.
int8 CONFIRMATION=2

# Holds one of the above constant values.
uint16 value


