package ihmc_msgs;

public interface HandTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## HandTrajectoryRosMessage\n# This message commands the controller to move in taskspace a hand to the desired pose (position &\n# orientation) while going through the specified trajectory points. A third order polynomial function\n# is used to interpolate positions and a hermite based curve (third order) is used to interpolate the\n# orientations. To excute a single straight line trajectory to reach a desired hand pose, set only one\n# trajectory point with zero velocity and its time to be equal to the desired trajectory time. A\n# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the\n# controller. This rule does not apply to the fields of this message.\n\n# Specifies which hand will execute the trajectory.\nuint8 robot_side\n\n# The position/orientation trajectory information.\nihmc_msgs/SE3TrajectoryRosMessage se3_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  ihmc_msgs.SE3TrajectoryRosMessage getSe3Trajectory();
  void setSe3Trajectory(ihmc_msgs.SE3TrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
