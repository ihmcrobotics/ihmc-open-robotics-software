package ihmc_msgs;

public interface HighLevelStateChangeStatusRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStateChangeStatusRosMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStateChangeStatusRosMessage\n# This message notifies the user of a change in the high level state. This message\'s primary use is to\n# signal a requested state change is completed.\n\n# initialState gives the controller\'s state prior to transition\nint8 initial_high_level_controller_name\n\n# endState gives the state the controller has transitioned into\nint8 end_high_level_controller_name\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getInitialHighLevelControllerName();
  void setInitialHighLevelControllerName(byte value);
  byte getEndHighLevelControllerName();
  void setEndHighLevelControllerName(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
