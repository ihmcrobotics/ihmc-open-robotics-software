package ihmc_msgs;

public interface GoHomeRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/GoHomeRosMessage";
  static final java.lang.String _DEFINITION = "## GoHomeRosMessage\n# The message commands the controller to bring the given part of the body back to a default\n# configuration called \'home\'. It is useful to get back to a safe configuration before walking.\n\n# Specifies the part of the body the user wants to move back to it home configuration.\nuint8 body_part\n\n# Needed to identify a side dependent end-effector.\nuint8 robot_side\n\n# How long the trajectory will spline from the current desired to the home configuration.\nfloat64 trajectory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"body_part\" enum values:\nuint8 ARM=0 # Request the chest to go back to a straight up configuration.\nuint8 CHEST=1 # Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.\nuint8 PELVIS=2 # Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte ARM = 0;
  static final byte CHEST = 1;
  static final byte PELVIS = 2;
  byte getBodyPart();
  void setBodyPart(byte value);
  byte getRobotSide();
  void setRobotSide(byte value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
