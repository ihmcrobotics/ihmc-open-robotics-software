package ihmc_msgs;

public interface AtlasElectricMotorEnablePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasElectricMotorEnablePacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasElectricMotorEnablePacketMessage\r\n# Specifies a specific electric motor in the Atlas forearm to power on or off.\r\n\r\n# Options for motorEnableEnum\r\nuint8 L_ARM_WRY=0 # left upper wrist pitch\r\nuint8 L_ARM_WRX=1 # left wrist roll\r\nuint8 L_ARM_WRY2=2 # left lower wrist pitch\r\nuint8 R_ARM_WRY=3 # right upper wrist pitch\r\nuint8 R_ARM_WRX=4 # right wrist roll\r\nuint8 R_ARM_WRY2=5 # right lower wrist pitch\r\nuint8 motor_enable_enum\r\n\r\nbool enable\r\n\r\n\r\n";
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
}
