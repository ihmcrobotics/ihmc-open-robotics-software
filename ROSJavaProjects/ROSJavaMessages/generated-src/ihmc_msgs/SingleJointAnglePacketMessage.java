package ihmc_msgs;

public interface SingleJointAnglePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SingleJointAnglePacketMessage";
  static final java.lang.String _DEFINITION = "## SingleJointAnglePacketMessage\r\n# This message contains a desired joint angle for a single joint.\r\n\r\nstring joint_name\r\n\r\nfloat64 angle\r\n\r\nfloat64 trajcetory_time\r\n\r\n\r\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getAngle();
  void setAngle(double value);
  double getTrajcetoryTime();
  void setTrajcetoryTime(double value);
}
