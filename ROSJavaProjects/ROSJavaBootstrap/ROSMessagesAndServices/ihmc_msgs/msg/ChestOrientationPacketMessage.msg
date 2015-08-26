## ChestOrientationPacketMessage
# This message sets the orientation of the robot's chest in world coordinates.

geometry_msgs/Quaternion orientation

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectory_time

# toHomePosition can be used to move the chest back to its default starting position
bool to_home_orientation

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


