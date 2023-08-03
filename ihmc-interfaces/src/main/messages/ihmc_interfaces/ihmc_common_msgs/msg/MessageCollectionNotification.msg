# This is part of the IHMC Common message package.
# Notifies the user that a MessageCollection has been received.
# This is used to know when the controller is ready to start collecting the actual collection of messages.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# This is the sequence ID of the MessageCollection that has just been received.
uint32 message_collection_sequence_id