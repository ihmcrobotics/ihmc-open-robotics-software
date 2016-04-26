package ihmc_msgs;

public interface OneDoFJointTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/OneDoFJointTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## OneDoFJointTrajectoryRosMessage\n# This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory. A third order polynomial function is used to interpolate between trajectory points.\n\n# List of trajectory points to go through while executing the trajectory.\nTrajectoryPoint1DRosMessage[] trajectory_points\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.TrajectoryPoint1DRosMessage> getTrajectoryPoints();
  void setTrajectoryPoints(java.util.List<ihmc_msgs.TrajectoryPoint1DRosMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}
