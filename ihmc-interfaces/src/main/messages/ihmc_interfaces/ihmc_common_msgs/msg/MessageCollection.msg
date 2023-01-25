# This is part of the IHMC Common message package.
# This message is used mainly with the IHMC whole-body controller.
# When the execution of a collection of messages is to be synchronized, these messages should be attributed
# a unique sequence ID. Then by sending beforehand a MessageCollection holding onto the sequence IDs of all these messages,
# the controller will wait to receive all the messages before processing them.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id
# The sequence IDs of all the messages that are expected to be received.
uint32[] sequences