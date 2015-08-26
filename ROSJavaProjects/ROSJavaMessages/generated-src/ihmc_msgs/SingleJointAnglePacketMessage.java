package ihmc_msgs;

public interface SingleJointAnglePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SingleJointAnglePacketMessage";
  static final java.lang.String _DEFINITION = "## SingleJointAnglePacketMessage\n# This message contains a desired joint angle for a single joint.\n\nstring joint_name\n\nfloat64 angle\n\nfloat64 trajcetory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.Only the unique id in the top level message is used, the unique id in nested messages is ignored.Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getAngle();
  void setAngle(double value);
  double getTrajcetoryTime();
  void setTrajcetoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
