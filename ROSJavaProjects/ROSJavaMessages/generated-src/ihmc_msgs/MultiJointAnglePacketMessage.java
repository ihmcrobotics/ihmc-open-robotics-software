package ihmc_msgs;

public interface MultiJointAnglePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/MultiJointAnglePacketMessage";
  static final java.lang.String _DEFINITION = "## MultiJointAnglePacketMessage\n# This message contains wraps up multiple SingleJointAnglePackets so they will beexecuted simultaneusly.\n\nSingleJointAnglePacketMessage[] single_joint_angle_packets\n\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.SingleJointAnglePacketMessage> getSingleJointAnglePackets();
  void setSingleJointAnglePackets(java.util.List<ihmc_msgs.SingleJointAnglePacketMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}
