package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage\n# for information about defining a footstep.\n\nFootstepDataMessage[] footstep_data_list\n\n# swingTime is the time spent in single-support when stepping\nfloat64 swing_time\n\n# transferTime is the time spent in double-support between steps\nfloat64 transfer_time\n\n\n";
  java.util.List<ihmc_msgs.FootstepDataMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataMessage> value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
}
