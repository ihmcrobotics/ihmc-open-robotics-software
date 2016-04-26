package ihmc_msgs;

public interface NeckTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/NeckTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## NeckTrajectoryRosMessage\n# This message commands the controller to move the neck in jointspace to the desired joint angles while going through the specified trajectory points. A third order polynomial function is used to interpolate between trajectory points. A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.\n\n# List of points in the trajectory. The expected joint ordering is from the closest joint to the chest to the closest joint to the head.\nOneDoFJointTrajectoryRosMessage[] joint_trajectory_messages\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.OneDoFJointTrajectoryRosMessage> getJointTrajectoryMessages();
  void setJointTrajectoryMessages(java.util.List<ihmc_msgs.OneDoFJointTrajectoryRosMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}
