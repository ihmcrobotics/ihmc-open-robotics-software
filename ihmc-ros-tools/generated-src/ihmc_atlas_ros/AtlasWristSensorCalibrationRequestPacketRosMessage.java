package ihmc_atlas_ros;

public interface AtlasWristSensorCalibrationRequestPacketRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_atlas_ros/AtlasWristSensorCalibrationRequestPacketRosMessage";
  static final java.lang.String _DEFINITION = "## AtlasWristSensorCalibrationRequestPacketRosMessage\n# Request taring of the wrist force/torque sensors.\n\n# The robot side (left or right) for the wrist sensor you would like to request calibration for.\nint8 robot_side\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
