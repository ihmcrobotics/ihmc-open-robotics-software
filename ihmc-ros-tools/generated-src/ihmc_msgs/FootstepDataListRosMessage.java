package ihmc_msgs;

public interface FootstepDataListRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListRosMessage\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for\n# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as\n# invalid and will not be processed by the controller. This rule does not apply to the fields of this\n# message.\n\n# Defines the list of footstep to perform.\nihmc_msgs/FootstepDataRosMessage[] footstep_data_list\n\n# When CONTROL_DURATIONS is chosen:  The controller will try to achieve the swingDuration and the\n# transferDuration specified in the message. If a  footstep touches down early, the next step will not\n# be affected by this and the whole trajectory might finish  earlier then expected. When\n# CONTROL_ABSOLUTE_TIMINGS is chosen:  The controller will compute the expected times for swing start\n# and touchdown and attempt to start a footstep  at that time. If a footstep touches down early, the\n# following transfer will be extended to make up for this  time difference and the footstep plan will\n# finish at the expected time.\nint8 execution_timing\n\n# The swingDuration is the time a foot is not in ground contact during a step. Each step in a list of\n# footsteps might have a different swing duration. The value specified here is a default value, used\n# if a footstep in this list was created without a swingDuration.\nfloat64 default_swing_duration\n\n# The transferDuration is the time spent with the feet in ground contact before a step. Each step in a\n# list of footsteps might have a different transfer duration. The value specified here is a default\n# value, used if a footstep in this list was created without a transferDuration.\nfloat64 default_transfer_duration\n\n# Specifies the time used to return to a stable standing stance after the execution of the footstep\n# list is finished. If the value is negative the defaultTransferDuration will be used.\nfloat64 final_transfer_duration\n\n# If false the controller adjust each footstep height to be at the support sole height.\nbool trust_height_of_footsteps\n\n# Contains information on whether the robot can automatically adjust its footsteps to retain balance.\nbool are_footsteps_adjustable\n\n# If true the controller will adjust upcoming footsteps with the location error of previous steps.\nbool offset_footsteps_with_execution_error\n\n# Properties for queueing footstep lists.\nihmc_msgs/QueueableRosMessage queueing_properties\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.FootstepDataRosMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataRosMessage> value);
  byte getExecutionTiming();
  void setExecutionTiming(byte value);
  double getDefaultSwingDuration();
  void setDefaultSwingDuration(double value);
  double getDefaultTransferDuration();
  void setDefaultTransferDuration(double value);
  double getFinalTransferDuration();
  void setFinalTransferDuration(double value);
  boolean getTrustHeightOfFootsteps();
  void setTrustHeightOfFootsteps(boolean value);
  boolean getAreFootstepsAdjustable();
  void setAreFootstepsAdjustable(boolean value);
  boolean getOffsetFootstepsWithExecutionError();
  void setOffsetFootstepsWithExecutionError(boolean value);
  ihmc_msgs.QueueableRosMessage getQueueingProperties();
  void setQueueingProperties(ihmc_msgs.QueueableRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
