package ihmc_msgs;

public interface HandDesiredConfigurationRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandDesiredConfigurationRosMessage";
  static final java.lang.String _DEFINITION = "## HandDesiredConfigurationRosMessage\n# Packet for commanding the hands to perform various predefined grasps. A message with a unique id\n# equals to 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Specifies the side of the robot that will execute the trajectory\nuint8 robot_side\n\n# Specifies the grasp to perform\nuint8 hand_desired_configuration\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"hand_configuration\" enum values:\nuint8 STOP=0 # stops the fingers at their current position\nuint8 OPEN=1 # fully opens the fingers\nuint8 CLOSE=2 # fully closes the fingers\nuint8 CRUSH=3 # fully closes the fingers applying maximum force\nuint8 HOOK=4 # closes all but one finger to create a hook\nuint8 BASIC_GRIP=5 # sets gripper to use a standard grasp\nuint8 PINCH_GRIP=6 # sets gripper to use a pinch grasp where the thumb and fingers come together when closed\nuint8 WIDE_GRIP=7 # sets gripper to use a wide-spread finger grasp\nuint8 SCISSOR_GRIP=8 # sets gripper to use a scissor grasp where the index and middle finger come together when closed\nuint8 RESET=9 # sets all fingers to their zero position\nuint8 OPEN_FINGERS=10 # fully open all fingers except the thumb\nuint8 OPEN_THUMB=11 # fully open the thumb only\nuint8 CLOSE_FINGERS=12 # fully close all fingers except the thumb\nuint8 CLOSE_THUMB=13 # fully close the thumb only\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte STOP = 0;
  static final byte OPEN = 1;
  static final byte CLOSE = 2;
  static final byte CRUSH = 3;
  static final byte HOOK = 4;
  static final byte BASIC_GRIP = 5;
  static final byte PINCH_GRIP = 6;
  static final byte WIDE_GRIP = 7;
  static final byte SCISSOR_GRIP = 8;
  static final byte RESET = 9;
  static final byte OPEN_FINGERS = 10;
  static final byte OPEN_THUMB = 11;
  static final byte CLOSE_FINGERS = 12;
  static final byte CLOSE_THUMB = 13;
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getHandDesiredConfiguration();
  void setHandDesiredConfiguration(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
