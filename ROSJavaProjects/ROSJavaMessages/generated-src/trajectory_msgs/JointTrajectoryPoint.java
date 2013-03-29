package trajectory_msgs;

public interface JointTrajectoryPoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "trajectory_msgs/JointTrajectoryPoint";
  static final java.lang.String _DEFINITION = "float64[] positions\nfloat64[] velocities\nfloat64[] accelerations\nduration time_from_start";
  double[] getPositions();
  void setPositions(double[] value);
  double[] getVelocities();
  void setVelocities(double[] value);
  double[] getAccelerations();
  void setAccelerations(double[] value);
  org.ros.message.Duration getTimeFromStart();
  void setTimeFromStart(org.ros.message.Duration value);
}
