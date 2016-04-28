package ihmc_msgs;

public interface WholeBodyTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WholeBodyTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## WholeBodyTrajectoryRosMessage\n# Send whole body trajectories to the robot. A best effort is made to execute the trajectory while\n# balance is kept.  A message with a unique id equals to 0 will be interpreted as invalid and will not\n# be processed by the controller. This rule DOES apply to the fields of this message. If setting a\n# field to null is not an option (going through IHMC ROS API), the user can use the latter rule to\n# select the messages to be processed by the controller.\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  long getUniqueId();
  void setUniqueId(long value);
}
