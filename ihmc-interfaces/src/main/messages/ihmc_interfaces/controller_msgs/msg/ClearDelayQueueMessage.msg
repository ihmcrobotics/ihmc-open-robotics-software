# This message is part of the IHMC whole-body controller API.
# This message is used to clear the delay buffers on the controller, if you sent a message with a
# delay and now you do not want them executed, use this command.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# If only a specific message type is to be cleared, use this field.
# The integer represents the hash code of the message simple name.
# The hash code of a string should be computed as in java.lang.String.hashCode().
int32 class_simple_name_based_hash_code
# If all the messages should be cleared, then this field should be set to true.
bool clear_all_delay_buffers