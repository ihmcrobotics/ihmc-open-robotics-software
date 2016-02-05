## SingleJointAnglePacketMessage
# This message contains a desired joint angle for a single joint.

string joint_name

float64 angle

float64 trajcetory_time

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


