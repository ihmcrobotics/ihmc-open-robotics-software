package ihmc_msgs;

public interface ComHeightPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ComHeightPacketMessage";
  static final java.lang.String _DEFINITION = "## ComHeightPacketMessage\r\n# This message sets the robot\'s center of mass height.\r\n\r\nfloat64 MIN_COM_HEIGHT\r\n\r\nfloat64 MAX_COM_HEIGHT\r\n\r\n# heightOffset specifies CoM height relative to the default starting height, which is\r\n# about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about\r\n# 68.9 cm above ground level\r\nfloat64 heightOffset\r\n\r\n# trajectoryTime specifies how fast or how slow to move to the desired pose\r\nfloat64 trajectoryTime\r\n\r\nint8 destination\r\n\r\n\r\n";
  double getMINCOMHEIGHT();
  void setMINCOMHEIGHT(double value);
  double getMAXCOMHEIGHT();
  void setMAXCOMHEIGHT(double value);
  double getHeightOffset();
  void setHeightOffset(double value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  byte getDestination();
  void setDestination(byte value);
}
