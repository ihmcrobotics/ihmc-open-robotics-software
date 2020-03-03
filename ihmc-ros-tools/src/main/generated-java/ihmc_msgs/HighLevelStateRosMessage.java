package ihmc_msgs;

public interface HighLevelStateRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStateRosMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStateRosMessage\n# This message is used to switch the control scheme between force and position control. WARNING: When\n# in position control, the IHMC balance algorithms will be disabled and it is up to the user to ensure\n# stability.\n\n# The enum value of the current high level state of the robot.\nint8 high_level_controller_name\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getHighLevelControllerName();
  void setHighLevelControllerName(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
