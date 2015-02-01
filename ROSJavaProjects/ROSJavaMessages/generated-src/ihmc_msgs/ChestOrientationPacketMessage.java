package ihmc_msgs;

public interface ChestOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ChestOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## ChestOrientationPacketMessage\r\n# This message gives the orientation of the robot\'s chest in world frame.\r\n\r\ngeometry_msgs/Quaternion orientation\r\nfloat64 trajectoryTime\r\n# toHomePosition can be used to move the Pelvis back to its starting position\r\nbool toHomeOrientation\r\n\r\n";
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  boolean getToHomeOrientation();
  void setToHomeOrientation(boolean value);
}
