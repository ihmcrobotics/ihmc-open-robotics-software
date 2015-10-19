package ihmc_msgs;

public interface AtlasWristSensorCalibrationRequestPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/AtlasWristSensorCalibrationRequestPacketMessage";
  static final java.lang.String _DEFINITION = "## AtlasWristSensorCalibrationRequestPacketMessage\n# Request taring of the wrist force/torque sensors.\n\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
