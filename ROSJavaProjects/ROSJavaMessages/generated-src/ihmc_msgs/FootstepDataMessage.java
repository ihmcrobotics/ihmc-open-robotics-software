package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\r\n# This message gives the position, orientation and side of a single footstep to the controller.\r\n# If sending a single footstep, position and orientation should be in pelvis frame.\r\n\r\n#Options for enum\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\ngeometry_msgs/Vector3 location\r\ngeometry_msgs/Quaternion orientation\r\nPoint2dMessage[] predictedContactPoints\r\n\r\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dMessage> value);
}
