package ihmc_msgs;

public interface FootLoadBearingRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootLoadBearingRosMessage";
  static final java.lang.String _DEFINITION = "## FootLoadBearingRosMessage\n# This message commands the controller to start loading a foot that was unloaded to support the robot\n# weight.  When the robot is performing a \'flamingo stance\' (one foot in the air not actually walking)\n# and the user wants the robot to switch back to double support. A message with a unique id equals to\n# 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Needed to identify a side dependent end-effector.\nint8 robot_side\n\n# Wether the end-effector should be loaded or unloaded.\nint8 load_bearing_request\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getLoadBearingRequest();
  void setLoadBearingRequest(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
