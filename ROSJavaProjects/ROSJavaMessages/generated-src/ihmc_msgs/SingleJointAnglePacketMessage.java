package ihmc_msgs;

public interface SingleJointAnglePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SingleJointAnglePacketMessage";
  static final java.lang.String _DEFINITION = "## SingleJointAnglePacketMessage\n# This message contains a desired joint angle for a single joint.\n\nstring joint_name\n\nfloat64 angle\n\nfloat64 trajcetory_time\n\n\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getAngle();
  void setAngle(double value);
  double getTrajcetoryTime();
  void setTrajcetoryTime(double value);
}
