package ihmc_msgs;

public interface LastReceivedMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LastReceivedMessage";
  static final java.lang.String _DEFINITION = "##LastReceivedMessage\n\nstring type\nint64 uniqueId\nint64 receiveTimestamp\nfloat64 timeSinceLastReceived\n";
  java.lang.String getType();
  void setType(java.lang.String value);
  long getUniqueId();
  void setUniqueId(long value);
  long getReceiveTimestamp();
  void setReceiveTimestamp(long value);
  double getTimeSinceLastReceived();
  void setTimeSinceLastReceived(double value);
}
