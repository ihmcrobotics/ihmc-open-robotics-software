package ihmc_msgs;

public interface GoHomeRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/GoHomeRosMessage";
  static final java.lang.String _DEFINITION = "## GoHomeRosMessage\n# The message commands the controller to bring the given part of the body back to a default\n# configuration called \'home\'. It is useful to get back to a safe configuration before walking.\n\n# Specifies the part of the body the user wants to move back to it home configuration.\nint8 humanoid_body_part\n\n# Needed to identify a side dependent end-effector.\nint8 robot_side\n\n# How long the trajectory will spline from the current desired to the home configuration.\nfloat64 trajectory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getHumanoidBodyPart();
  void setHumanoidBodyPart(byte value);
  byte getRobotSide();
  void setRobotSide(byte value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
