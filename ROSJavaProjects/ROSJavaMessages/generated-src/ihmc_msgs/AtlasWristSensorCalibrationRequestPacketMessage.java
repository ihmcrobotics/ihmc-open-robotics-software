package ihmc_msgs;

public interface AtlasWristSensorCalibrationRequestPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasWristSensorCalibrationRequestPacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasWristSensorCalibrationRequestPacketMessage\r\n# Request taring of the wrist force/torque sensors.\r\n\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\n\r\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
}
