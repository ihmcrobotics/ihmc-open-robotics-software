package ihmc_msgs;

public interface HandDesiredConfigurationRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandDesiredConfigurationRosMessage";
  static final java.lang.String _DEFINITION = "## HandDesiredConfigurationRosMessage\n# Packet for commanding the hands to perform various predefined grasps. A message with a unique id\n# equals to 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Specifies the side of the robot that will execute the trajectory\nint8 robot_side\n\n# Specifies the grasp to perform\nint8 desired_hand_configuration\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getDesiredHandConfiguration();
  void setDesiredHandConfiguration(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
