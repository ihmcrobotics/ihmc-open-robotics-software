package ihmc_msgs;

public interface ChestOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ChestOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## ChestOrientationPacketMessage\r\n# This message gives the orientation of the robot\'s chest in world frame.\r\n\r\ngeometry_msgs/Quaternion quaternion\r\n\r\n";
  geometry_msgs.Quaternion getQuaternion();
  void setQuaternion(geometry_msgs.Quaternion value);
}
