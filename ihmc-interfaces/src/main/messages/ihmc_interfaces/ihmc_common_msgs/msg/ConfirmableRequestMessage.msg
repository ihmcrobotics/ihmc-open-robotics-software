# A way of avoiding local data get overriden before
# it is received by peers.

# Nothing special has happened. This should trigger no effects.
byte NOOP = 0
# This message is a request to change something.
byte REQUEST = 1
# Confirmation that the request has been received by the other side.
byte CONFIRMATION = 2

# Holds one of the above constant values.
uint16 value