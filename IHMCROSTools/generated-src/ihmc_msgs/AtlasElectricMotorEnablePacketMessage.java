package ihmc_msgs;

public interface AtlasElectricMotorEnablePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasElectricMotorEnablePacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasElectricMotorEnablePacketMessage\n# Specifies a specific electric motor in the Atlas forearm to power on or off.\n\n# Options for motorEnableEnum\nuint8 L_ARM_WRY=0 # left upper wrist pitch\nuint8 L_ARM_WRX=1 # left wrist roll\nuint8 L_ARM_WRY2=2 # left lower wrist pitch\nuint8 R_ARM_WRY=3 # right upper wrist pitch\nuint8 R_ARM_WRX=4 # right wrist roll\nuint8 R_ARM_WRY2=5 # right lower wrist pitch\nuint8 motor_enable_enum\n\nbool enable\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
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
