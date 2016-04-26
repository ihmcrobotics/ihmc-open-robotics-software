package ihmc_msgs;

public interface SingleJointAnglePacketRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SingleJointAnglePacketRosMessage";
  static final java.lang.String _DEFINITION = "## SingleJointAnglePacketRosMessage\n# This message contains a desired joint angle for a single joint.\n\nstring joint_name\n\nfloat64 angle\n\nfloat64 trajcetory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getAngle();
  void setAngle(double value);
  double getTrajcetoryTime();
  void setTrajcetoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
