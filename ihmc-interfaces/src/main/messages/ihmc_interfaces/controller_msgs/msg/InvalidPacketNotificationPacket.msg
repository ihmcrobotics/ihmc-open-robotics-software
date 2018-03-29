# This message notifies the user that a previously sent message was rejected by the whole-body controller.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

string packet_class_simple_name
string error_message