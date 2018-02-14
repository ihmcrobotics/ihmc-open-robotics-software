package ihmc_msgs;

public interface FootTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## FootTrajectoryRosMessage\n# This message commands the controller first to unload if necessary and then to move in taskspace a\n# foot to the desired pose (position & orientation) while going through the specified trajectory\n# points. A third order polynomial function is used to interpolate positions and a hermite based curve\n# (third order) is used to interpolate the orientations. To excute a single straight line trajectory\n# to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be\n# equal to the desired trajectory time. A message with a unique id equals to 0 will be interpreted as\n# invalid and will not be processed by the controller. This rule does not apply to the fields of this\n# message.\n\n# Specifies which foot will execute the trajectory.\nuint8 robot_side\n\n# The position/orientation trajectory information.\nihmc_msgs/SE3TrajectoryRosMessage se3_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  ihmc_msgs.SE3TrajectoryRosMessage getSe3Trajectory();
  void setSe3Trajectory(ihmc_msgs.SE3TrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}
