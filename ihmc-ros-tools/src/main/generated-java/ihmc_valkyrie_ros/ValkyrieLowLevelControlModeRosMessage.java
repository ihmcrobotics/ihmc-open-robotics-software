package ihmc_valkyrie_ros;

public interface ValkyrieLowLevelControlModeRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage";
  static final java.lang.String _DEFINITION = "## ValkyrieLowLevelControlModeRosMessage\n# Request a Valkyrie low-level control mode, which is either: stand-prep, calibration, or high-level\n# control.\n\n# Specifies the low-level control mode to switch to.\nuint8 requested_control_mode\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"control_mode\" enum values:\nuint8 STAND_PREP=0 # Simple position controller to keep the robot in a \'ready-to-walk\' configuration.\nuint8 CALIBRATION=1 # Automated calibration routine to estimate the joint torque offsets and foot force/torque sensor offsets. The routine takes about 15 seconds.\nuint8 HIGH_LEVEL_CONTROL=2 # Switching to the high level walking controller.\n\n";
  static final byte STAND_PREP = 0;
  static final byte CALIBRATION = 1;
  static final byte HIGH_LEVEL_CONTROL = 2;
  byte getRequestedControlMode();
  void setRequestedControlMode(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
