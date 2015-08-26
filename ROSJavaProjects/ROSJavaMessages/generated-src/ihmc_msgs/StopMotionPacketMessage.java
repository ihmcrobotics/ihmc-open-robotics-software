package ihmc_msgs;

public interface StopMotionPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/StopMotionPacketMessage";
  static final java.lang.String _DEFINITION = "## StopMotionPacketMessage\n# Stop the execution of a Whole Body Trajectory\n\nint64 unique_id\n\n\n";
  long getUniqueId();
  void setUniqueId(long value);
}
