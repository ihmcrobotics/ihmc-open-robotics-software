package ihmc_msgs;

public interface ArmDesiredAccelerationsRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmDesiredAccelerationsRosMessage";
  static final java.lang.String _DEFINITION = "## ArmDesiredAccelerationsRosMessage\n# This message gives the user the option to bypass IHMC feedback controllers for the arm joints by\n# sending desired arm joint accelerations. One needs experience in control when activating the bypass\n# as it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id\n# equals to 0 will be interpreted as invalid and will not be processed by the controller.\n\n# Specifies the side of the robot that will execute the trajectory.\nuint8 robot_side\n\n# The desired joint acceleration information.\nihmc_msgs/DesiredAccelerationsRosMessage desired_accelerations\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  ihmc_msgs.DesiredAccelerationsRosMessage getDesiredAccelerations();
  void setDesiredAccelerations(ihmc_msgs.DesiredAccelerationsRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
