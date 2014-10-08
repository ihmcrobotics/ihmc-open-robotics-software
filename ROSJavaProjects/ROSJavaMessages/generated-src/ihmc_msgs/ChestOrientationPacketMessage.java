package ihmc_msgs;

public interface ChestOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ChestOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "# ChestOrientationPacketMessage\n\ngeometry_msgs/Quaternion quaternion\n\n";
  geometry_msgs.Quaternion getQuaternion();
  void setQuaternion(geometry_msgs.Quaternion value);
}
