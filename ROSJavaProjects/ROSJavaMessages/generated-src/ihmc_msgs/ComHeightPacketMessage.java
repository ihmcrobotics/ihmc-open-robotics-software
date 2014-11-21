package ihmc_msgs;

public interface ComHeightPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ComHeightPacketMessage";
  static final java.lang.String _DEFINITION = "## ComHeightPacketMessage\r\n# This message sets the robot\'s center of mass height.\r\n\r\nfloat64 MIN_COM_HEIGHT\r\nfloat64 MAX_COM_HEIGHT\r\n# heightOffset specifies CoM height relative to the default starting height\r\nfloat64 heightOffset\r\n\r\n";
  double getMINCOMHEIGHT();
  void setMINCOMHEIGHT(double value);
  double getMAXCOMHEIGHT();
  void setMAXCOMHEIGHT(double value);
  double getHeightOffset();
  void setHeightOffset(double value);
}
