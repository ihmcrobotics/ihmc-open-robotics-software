package ihmc_msgs;

public interface FootPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootPosePacketMessage";
  static final java.lang.String _DEFINITION = "## FootPosePacketMessage\n# This message gives the position and oreientation of one foot in world coordinates.\n\n#Options for enum\n# uint8 L = 0\n# uint8 R = 1\nuint8 robotSide\ngeometry_msgs/Vector3 position\ngeometry_msgs/Quaternion orientation\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
}
