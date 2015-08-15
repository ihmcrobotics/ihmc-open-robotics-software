package ihmc_msgs;

public interface JointTrajectoryPointMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/JointTrajectoryPointMessage";
  static final java.lang.String _DEFINITION = "## JointTrajectoryPointMessage\r\n# Sub-Packet for a point in an arm joint trajectory. It works similar to the\r\n# trajectory_msgs/JointTrajectoryPoint message.\r\n\r\n# Arm joint angles for this waypoint in order. For Atlas the controller assumes\r\n# joint angles will be given in the following order: shoulder pitch, shoulder\r\n# roll, elbow pitch, elbow roll, wrist pitch, wrist roll\r\nfloat64[] positions\r\n\r\n# Arm joint angular velocities for this waypoint in order.\r\nfloat64[] velocities\r\n\r\n# Time at which the waypoint is reached after the controller recieved the packet.\r\n# A value of zero is not allowed.\r\nfloat64 time\r\n\r\n\r\n";
  double[] getPositions();
  void setPositions(double[] value);
  double[] getVelocities();
  void setVelocities(double[] value);
  double getTime();
  void setTime(double value);
}
