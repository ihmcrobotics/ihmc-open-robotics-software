package ihmc_msgs;

public interface FootPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootPosePacketMessage";
  static final java.lang.String _DEFINITION = "## FootPosePacketMessage\r\n# This message gives the position and oreientation of one foot in world coordinates.\r\n\r\n#Options for enum\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\ngeometry_msgs/Vector3 position\r\ngeometry_msgs/Quaternion orientation\r\n\r\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
}
