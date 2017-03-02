package ihmc_msgs;

public interface EndEffectorLoadBearingRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/EndEffectorLoadBearingRosMessage";
  static final java.lang.String _DEFINITION = "## EndEffectorLoadBearingRosMessage\n# This message commands the controller to start loading an end effector that was unloaded to support\n# the robot weight.  One application is making a foot loadbearing. When the robot is performing a\n# \'flamingo stance\' (one foot in the air not actually walking) and the user wants the robot to switch\n# back to double support. A message with a unique id equals to 0 will be interpreted as invalid and\n# will not be processed by the controller.\n\n# Needed to identify a side dependent end-effector.\nuint8 robot_side\n\n# Specifies which end-effector should be loaded/unloaded.\nuint8 end_effector\n\n# Wether the end-effector should be loaded or unloaded.\nuint8 request\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"end_effector\" enum values:\nuint8 FOOT=0 # In this case, the user also needs to provide the robotSide. If in the air, the corresponding foot will enter first a vertical ground approach transition and eventually touch the ground and switch to loadbearing. Then the robot is ready to walk. It is preferable to request a foot to switch to load bearing when it is aready close to the ground.\nuint8 HAND=1 # In this case, the user also needs to provide the robotSide. It is preferable to request a hand to switch to load bearing when it is aready close to the ground.\n\n# \"load_bearing_request\" enum values:\nuint8 LOAD=0 # Request to load the given end-effector.\nuint8 UNLOAD=1 # Request to unload the given end-effector.\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte FOOT = 0;
  static final byte HAND = 1;
  static final byte LOAD = 0;
  static final byte UNLOAD = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getEndEffector();
  void setEndEffector(byte value);
  byte getRequest();
  void setRequest(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
