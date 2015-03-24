package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\n# world frame\n\n# Options for robotSide\n# LEFT = 0 - refers to the LEFT side of a robot\n# RIGHT = 1 - refers to the RIGHT side of a robot\nuint8 robot_side\n\ngeometry_msgs/Vector3 location\n\ngeometry_msgs/Quaternion orientation\n\n# This contains information on what the swing trajectory should be for each step\n# Options for trajectoryType\n# DEFAULT = 0 - is a default trajectory\n# BASIC = 1 - will do a basic swing with the specified swing height\n# PUSH_RECOVERY = 2 - uses a low swing height for fast steps\n# OBSTACLE_CLEARANCE = 3 - will attempt to step over an obstacle\nuint8 trajectory_type\n\n# Contains information on how high the robot should step\nfloat64 swing_height\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  byte getTrajectoryType();
  void setTrajectoryType(byte value);
  double getSwingHeight();
  void setSwingHeight(double value);
}
