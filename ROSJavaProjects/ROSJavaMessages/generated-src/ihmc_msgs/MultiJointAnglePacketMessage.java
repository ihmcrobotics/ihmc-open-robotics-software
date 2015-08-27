package ihmc_msgs;

public interface MultiJointAnglePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/MultiJointAnglePacketMessage";
  static final java.lang.String _DEFINITION = "## MultiJointAnglePacketMessage\n# This message contains wraps up multiple SingleJointAnglePackets so they will beexecuted simultaneusly.\n\nSingleJointAnglePacketMessage[] single_joint_angle_packets\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.SingleJointAnglePacketMessage> getSingleJointAnglePackets();
  void setSingleJointAnglePackets(java.util.List<ihmc_msgs.SingleJointAnglePacketMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}
