package ihmc_msgs;

public interface MultiJointAnglePacketRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/MultiJointAnglePacketRosMessage";
  static final java.lang.String _DEFINITION = "## MultiJointAnglePacketRosMessage\n# This message contains wraps up multiple SingleJointAnglePackets so they will beexecuted\n# simultaneusly.\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  long getUniqueId();
  void setUniqueId(long value);
}
