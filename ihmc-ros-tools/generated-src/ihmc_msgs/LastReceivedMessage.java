package ihmc_msgs;

public interface LastReceivedMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LastReceivedMessage";
  static final java.lang.String _DEFINITION = "## LastReceivedMessage\n# Last Received Message echo\'s back the ID and type of the last message received by the IHMC ROS API.\n# The type of the last message received\nstring type\n\n# The Unique ID of the last message received.\nint64 unique_id\n\n# The timestamp at which the message was received.\nint64 receive_timestamp\n\n# The time since a message was received\nfloat64 time_since_last_received\n\n";
  java.lang.String getType();
  void setType(java.lang.String value);
  long getUniqueId();
  void setUniqueId(long value);
  long getReceiveTimestamp();
  void setReceiveTimestamp(long value);
  double getTimeSinceLastReceived();
  void setTimeSinceLastReceived(double value);
}
