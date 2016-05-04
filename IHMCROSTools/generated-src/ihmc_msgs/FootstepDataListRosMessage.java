package ihmc_msgs;

public interface FootstepDataListRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListRosMessage\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for\n# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as\n# invalid and will not be processed by the controller. This rule does not apply to the fields of this\n# message.\n\n# Defines the list of footstep to perform.\nihmc_msgs/FootstepDataRosMessage[] footstep_data_list\n\n# swingTime is the time spent in single-support when stepping\nfloat64 swing_time\n\n# transferTime is the time spent in double-support between steps\nfloat64 transfer_time\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.FootstepDataRosMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataRosMessage> value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
