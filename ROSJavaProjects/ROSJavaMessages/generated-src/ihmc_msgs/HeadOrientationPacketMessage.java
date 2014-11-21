package ihmc_msgs;

public interface HeadOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HeadOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## HeadOrientationPacketMessage\r\n# This message gives the desired head orientation of the robot.\r\n\r\n# quaternion gives the desired final orientation of the head\r\ngeometry_msgs/Quaternion quaternion\r\n\r\n";
  geometry_msgs.Quaternion getQuaternion();
  void setQuaternion(geometry_msgs.Quaternion value);
}
