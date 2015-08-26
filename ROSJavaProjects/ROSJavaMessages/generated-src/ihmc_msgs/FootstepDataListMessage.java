package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage\n# for information about defining a footstep.\n\nFootstepDataMessage[] footstep_data_list\n\n# swingTime is the time spent in single-support when stepping\nfloat64 swing_time\n\n# transferTime is the time spent in double-support between steps\nfloat64 transfer_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.Only the unique id in the top level message is used, the unique id in nested messages is ignored.Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.FootstepDataMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataMessage> value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
