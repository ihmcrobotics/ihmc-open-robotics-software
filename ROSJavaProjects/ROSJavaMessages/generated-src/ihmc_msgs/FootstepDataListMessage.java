package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\r\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage\r\n# for information about defining a footstep.\r\n\r\nFootstepDataMessage[] footstep_data_list\r\n\r\n# swingTime is the time spent in single-support when stepping\r\nfloat64 swing_time\r\n\r\n# transferTime is the time spent in double-support between steps\r\nfloat64 transfer_time\r\n\r\n\r\n";
  java.util.List<ihmc_msgs.FootstepDataMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataMessage> value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
}
