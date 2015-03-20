package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\n# world frame\n\n#Options for robotSide\n# uint8 L = 0\n# uint8 R = 1\nuint8 robot_side\n\ngeometry_msgs/Vector3 location\n\ngeometry_msgs/Quaternion orientation\n\n# predictedContactPoints gives the vertices of the expected contact polygon between the foot and\n# the world. A value of null will use the default controller contact points\nPoint2dMessage[] predicted_contact_points\n\n# This contains information on what the swing trajectory should be for each step\n# DEFAULT gives the standard IHMC parabolic footstep trajectory\n# LOW_HEIGHT gives a lower-arched trajectory\n# STEP_ON_OR_OFF gives a higher-arched trajectory meant for stepping onto or off obstacles\n# BY_BOX gives a square-wave type trajectory according to the parameters of trajectoryBoxData\n# NO_STEP results in a null footstep. This probably should not be used outside of IHMC source\n#Options for trajectoryType\n# uint8 DEFAULT = 0\n# uint8 BASIC = 1\n# uint8 PUSH_RECOVERY = 2\n# uint8 OBSTACLE_CLEARANCE = 3\nuint8 trajectory_type\n\n# Contains information on how high the robot should step\nfloat64 swing_height\n\n\n";
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
