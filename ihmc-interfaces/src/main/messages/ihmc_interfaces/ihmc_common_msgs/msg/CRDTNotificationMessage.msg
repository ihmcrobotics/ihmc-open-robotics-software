# A way of sending a notification via CRDT synced state messages.
# By using 3 values, we can set a notification and receive a
# confirmation that the other side polled it without sync errors.

# Nothing special has happened. This should have no effect of subscribers.
byte NOOP = 0
# A set of the notification on the other side has been requested.
byte SET_REQUEST = 1
# Confirmation that the notification has been polled by the other side.
byte POLL_CONFIRMED = 2

# Holds one of the above constant values.
uint16 value