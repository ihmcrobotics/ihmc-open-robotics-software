package ihmc_msgs;

public interface PelvisOrientationTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PelvisOrientationTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## PelvisOrientationTrajectoryRosMessage\n# This message commands the controller to move in taskspace the pelvis to the desired orientation\n# while going through the specified trajectory points. A hermite based curve (third order) is used to\n# interpolate the orientations. This message allows controlling the pelvis orientation without\n# interferring with position that will still be controlled to maintain the current desired capture\n# poit position. To excute a normal trajectory to reach a desired pelvis orientation, set only one\n# trajectory point with zero velocity and its time to be equal to the desired trajectory time. A\n# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the\n# controller. This rule does not apply to the fields of this message.\n\n# The orientation trajectory information.\nihmc_msgs/SO3TrajectoryRosMessage so3_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  ihmc_msgs.SO3TrajectoryRosMessage getSo3Trajectory();
  void setSo3Trajectory(ihmc_msgs.SO3TrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
