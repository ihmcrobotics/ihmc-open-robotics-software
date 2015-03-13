package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\r\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\r\n# world frame\r\n\r\n#Options for robotSide\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\n\r\ngeometry_msgs/Vector3 location\r\n\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# predictedContactPoints gives the vertices of the expected contact polygon between the foot and\r\n# the world. A value of null will use the default controller contact points\r\nPoint2dMessage[] predictedContactPoints\r\n\r\n# This contains information on what the swing trajectory should be for each step\r\n# DEFAULT gives the standard IHMC parabolic footstep trajectory\r\n# LOW_HEIGHT gives a lower-arched trajectory\r\n# STEP_ON_OR_OFF gives a higher-arched trajectory meant for stepping onto or off obstacles\r\n# BY_BOX gives a square-wave type trajectory according to the parameters of trajectoryBoxData\r\n# NO_STEP results in a null footstep. This probably should not be used outside of IHMC source\r\n#Options for trajectoryType\r\n# uint8 DEFAULT = 0\r\n# uint8 BASIC = 1\r\n# uint8 PUSH_RECOVERY = 2\r\n# uint8 OBSTACLE_CLEARANCE = 3\r\nuint8 trajectoryType\r\n\r\n# Contains information on how high the robot should step\r\nfloat64 swingHeight\r\n\r\nint8 destination\r\n\r\n\r\n";
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
  byte getDestination();
  void setDestination(byte value);
}
