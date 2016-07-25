package ihmc_atlas_ros;

public interface AtlasElectricMotorEnablePacketRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_atlas_ros/AtlasElectricMotorEnablePacketRosMessage";
  static final java.lang.String _DEFINITION = "## AtlasElectricMotorEnablePacketRosMessage\n# Specifies a specific electric motor in the Atlas forearm to power on or off.\n\n# The Enum value of the motor to enable\nuint8 motor_enable_enum\n\n# Boolean for enable state; true for enable, false for disable.\nbool enable\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"atlas_electric_motor_packet_enum\" enum values:\nuint8 L_ARM_WRY=0 # left wrist roll\nuint8 L_ARM_WRX=1 # left upper wrist pitch\nuint8 L_ARM_WRY2=2 # left lower wrist pitch\nuint8 R_ARM_WRY=3 # right wrist roll\nuint8 R_ARM_WRX=4 # right upper wrist pitch\nuint8 R_ARM_WRY2=5 # right lower wrist pitch\n\n";
  static final byte L_ARM_WRY = 0;
  static final byte L_ARM_WRX = 1;
  static final byte L_ARM_WRY2 = 2;
  static final byte R_ARM_WRY = 3;
  static final byte R_ARM_WRX = 4;
  static final byte R_ARM_WRY2 = 5;
  byte getMotorEnableEnum();
  void setMotorEnableEnum(byte value);
  boolean getEnable();
  void setEnable(boolean value);
  long getUniqueId();
  void setUniqueId(long value);
}
