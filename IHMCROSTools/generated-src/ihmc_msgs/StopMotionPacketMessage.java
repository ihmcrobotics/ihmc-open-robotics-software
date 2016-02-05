package ihmc_msgs;

public interface StopMotionPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/StopMotionPacketMessage";
  static final java.lang.String _DEFINITION = "## StopMotionPacketMessage\n# Stop the execution of a Whole Body Trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  long getUniqueId();
  void setUniqueId(long value);
}
