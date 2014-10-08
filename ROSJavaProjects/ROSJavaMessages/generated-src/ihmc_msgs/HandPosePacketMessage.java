package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "# HandPosePacketMessage\n\n#Options for enum# uint8 L = 0\n# uint8 R = 1\nuint8 robotSide\n#Options for enum# uint8 CHEST = 0\n# uint8 WORLD = 1\nuint8 referenceFrame\n#Options for enum# uint8 HAND_POSE = 0\n# uint8 JOINT_ANGLES = 1\nuint8 dataType\nbool toHomePosition\ngeometry_msgs/Vector3 position\ngeometry_msgs/Quaternion orientation\nfloat64 trajectoryTime\nfloat64[] jointAngles\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getReferenceFrame();
  void setReferenceFrame(byte value);
  byte getDataType();
  void setDataType(byte value);
  boolean getToHomePosition();
  void setToHomePosition(boolean value);
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  double[] getJointAngles();
  void setJointAngles(double[] value);
}
