package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\r\n# This message specifies the position, orientation and side (left or right) of a desired footstep in\r\n# world frame\r\n\r\n#Options for robotSide\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\n\r\ngeometry_msgs/Vector3 location\r\n\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# predictedContactPoints gives the vertices of the expected contact polygon between the foot and\r\n# the world. A value of null will use the default controller contact points\r\nPoint2dMessage[] predictedContactPoints\r\n\r\n\r\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dMessage> value);
}
