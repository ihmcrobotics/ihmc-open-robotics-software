package ihmc_msgs;

public interface ChestTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ChestTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## ChestTrajectoryRosMessage\n# This message commands the controller to move in taskspace the chest to the desired orientation while\n# going through the specified trajectory points. A hermite based curve (third order) is used to\n# interpolate the orientations. To excute a simple trajectory to reach a desired chest orientation,\n# set only one trajectory point with zero velocity and its time to be equal to the desired trajectory\n# time. A message with a unique id equals to 0 will be interpreted as invalid and will not be\n# processed by the controller. This rule does not apply to the fields of this message.\n\n# The orientation trajectory information.\nihmc_msgs/SO3TrajectoryRosMessage so3_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  ihmc_msgs.SO3TrajectoryRosMessage getSo3Trajectory();
  void setSo3Trajectory(ihmc_msgs.SO3TrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
