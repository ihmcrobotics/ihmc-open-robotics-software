package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\r\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\r\n# world frame\r\n\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\ngeometry_msgs/Vector3 location\r\n\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\r\n# the world. A value of null will default to using the entire foot. Contact points should be specified\r\n# in foot sole frame, where the origin is at the center of the foot. Order of the points does not matter.\r\n# For example: to tell the controller to use the entire foot, the predicted contact points would be:\r\n# predicted_contact_points:\r\n# - {x: 0.5 * foot_length, y: -0.5 * toe_width}\r\n# - {x: 0.5 * foot_length, y: 0.5 * toe_width}\r\n# - {x: -0.5 * foot_length, y: -0.5 * heel_width}\r\n# - {x: -0.5 * foot_length, y: 0.5 * heel_width}\r\nPoint2dMessage[] predicted_contact_points\r\n\r\n# This contains information on what the swing trajectory should be for each step. Recomended is to default to basic.\r\n# Options for trajectoryType\r\nuint8 DEFAULT=0 # is a default trajectory\r\nuint8 BASIC=1 # will do a basic swing with the specified swing height\r\nuint8 PUSH_RECOVERY=2 # uses a low swing height for fast steps\r\nuint8 OBSTACLE_CLEARANCE=3 # will attempt to step over an obstacle\r\nuint8 trajectory_type\r\n\r\n# Contains information on how high the robot should step. This affects only basic and obstacle clearance trajectories.Recommended values are between 0.1 (default) and 0.25.\r\nfloat64 swing_height\r\n\r\n\r\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte DEFAULT = 0;
  static final byte BASIC = 1;
  static final byte PUSH_RECOVERY = 2;
  static final byte OBSTACLE_CLEARANCE = 3;
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dMessage> value);
  byte getTrajectoryType();
  void setTrajectoryType(byte value);
  double getSwingHeight();
  void setSwingHeight(double value);
}
