# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If only a specific message type is to be cleared, use this field.
# The integer represents the hash code of the message simple name.
# The hash code of a string should be computed as in java.lang.String.hashCode().
int32 class_simple_name_based_hash_code

# If all the messages should be cleared, then this field should be set to true.
bool clear_all_delay_buffers


