package ihmc_msgs;

public interface TrajectoryPoint1DRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/TrajectoryPoint1DRosMessage";
  static final java.lang.String _DEFINITION = "## TrajectoryPoint1DRosMessage\n# This class is used to build 1D trajectory messages including jointspace trajectory messages. For 3D\n# trajectory points look at EuclideanTrajectoryMessage (translational), SO3TrajectoryPointMessage\n# (rotational), and SE3TrajectoryPointMessage (translational AND rotational).\n\n# Time at which the trajectory point has to be reached. The time is relative to when the trajectory\n# starts.\nfloat64 time\n\n# Define the desired 1D position to be reached at this trajectory point.\nfloat64 position\n\n# Define the desired 1D velocity to be reached at this trajectory point.\nfloat64 velocity\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  double getTime();
  void setTime(double value);
  double getPosition();
  void setPosition(double value);
  double getVelocity();
  void setVelocity(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
