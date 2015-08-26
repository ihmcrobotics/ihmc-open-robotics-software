package ihmc_msgs;

public interface JointTrajectoryPointMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/JointTrajectoryPointMessage";
  static final java.lang.String _DEFINITION = "## JointTrajectoryPointMessage\n# Sub-Packet for a point in an arm joint trajectory. It works similar to the\n# trajectory_msgs/JointTrajectoryPoint message.\n\n# Arm joint angles for this waypoint in order. For Atlas the controller assumes\n# joint angles will be given in the following order: shoulder pitch, shoulder\n# roll, elbow pitch, elbow roll, wrist pitch, wrist roll\nfloat64[] positions\n\n# Arm joint angular velocities for this waypoint in order.\nfloat64[] velocities\n\n# Time at which the waypoint is reached after the controller recieved the packet.\n# A value of zero is not allowed.\nfloat64 time\n\nint64 unique_id\n\n\n";
  double[] getPositions();
  void setPositions(double[] value);
  double[] getVelocities();
  void setVelocities(double[] value);
  double getTime();
  void setTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
