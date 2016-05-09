package ihmc_msgs;

public interface FootstepDataListRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListRosMessage\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for\n# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as\n# invalid and will not be processed by the controller. This rule does not apply to the fields of this\n# message.\n\n# Defines the list of footstep to perform.\nihmc_msgs/FootstepDataRosMessage[] footstep_data_list\n\n# When OVERRIDE is chosen:  - All previously sent footstep lists will be overriden, regardless of\n# their execution mode  - A footstep can be overridden up to end of transfer. Once the swing foot\n# leaves the ground it cannot be overridden  When QUEUE is chosen:  This footstep list will be queued\n# with previously sent footstep lists, regardless of their execution mode  - The trajectory time and\n# swing time of all queued footsteps will be overwritten with this message\'s values.\nuint8 execution_mode\n\n# swingTime is the time spent in single-support when stepping  - Queueing does not support varying\n# swing times. If execution mode is QUEUE, all queued footsteps\' swingTime will be overwritten\nfloat64 swing_time\n\n# transferTime is the time spent in double-support between steps  - Queueing does not support varying\n# transfer times. If execution mode is QUEUE, all queued footsteps\' transferTime will be overwritten\nfloat64 transfer_time\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"execution_mode\" enum values:\nuint8 OVERRIDE=0 # This message will override the previous.\nuint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.\n\n";
  static final byte OVERRIDE = 0;
  static final byte QUEUE = 1;
  java.util.List<ihmc_msgs.FootstepDataRosMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataRosMessage> value);
  byte getExecutionMode();
  void setExecutionMode(byte value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
