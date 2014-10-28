package ihmc_msgs;

public interface FootstepDataMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataMessage\n# This message gives the position, orientation and side of a single footstep to the controller.\n# If sending a single footstep, position and orientation should be in pelvis frame.\n\n#Options for enum\n# uint8 L = 0\n# uint8 R = 1\nuint8 robotSide\ngeometry_msgs/Vector3 location\ngeometry_msgs/Quaternion orientation\nPoint2dMessage[] predictedContactPoints\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getLocation();
  void setLocation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  java.util.List<ihmc_msgs.Point2dMessage> getPredictedContactPoints();
  void setPredictedContactPoints(java.util.List<ihmc_msgs.Point2dMessage> value);
}
