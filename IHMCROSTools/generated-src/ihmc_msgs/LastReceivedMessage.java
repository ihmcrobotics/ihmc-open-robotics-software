package ihmc_msgs;

public interface LastReceivedMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LastReceivedMessage";
  static final java.lang.String _DEFINITION = "##LastReceivedMessage\n# Feedback from the last command received by the controller\n\n# Message type of the last command message received by the controller\nstring type\n\n# id of the last command message received by the controller receive\nint64 unique_id\n\n# timestamp in nanoseconds the controller received the command message with unique_id\nint64 receive_timestamp\n\n# time in nanoseconds since the command was received. The current time equals time_since_last_received + receive_timestamp\nint64 time_since_last_received\n";
  java.lang.String getType();
  void setType(java.lang.String value);
  long getUniqueId();
  void setUniqueId(long value);
  long getReceiveTimestamp();
  void setReceiveTimestamp(long value);
  long getTimeSinceLastReceived();
  void setTimeSinceLastReceived(long value);
}
