package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\r\n# This message commands the controller to move a hand to a given position\r\n#  and orientation. The position/orientation may be specified in world\r\n# frame, chest frame, or desired joint angles.\r\n\r\n#Options for enum\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\n#Options for enum\r\n# uint8 HAND_POSE = 0\r\n# uint8 JOINT_ANGLES = 1\r\nuint8 dataType\r\n#Options for enum\r\n# uint8 CHEST = 0\r\n# uint8 WORLD = 1\r\nuint8 referenceFrame\r\n# toHomePosition can be used to move the hands back to their starting position: down and beside the robot\r\nbool toHomePosition\r\ngeometry_msgs/Vector3 position\r\ngeometry_msgs/Quaternion orientation\r\n# trajectoryTime specifies how fast to move the hand to the desired position\r\nfloat64 trajectoryTime\r\n# jointAngles specifies the desired individual arm joint angles. For Atlas, those are:\r\n# shoulder pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll\r\nfloat64[] jointAngles\r\n\r\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  byte getDataType();
  void setDataType(byte value);
  byte getReferenceFrame();
  void setReferenceFrame(byte value);
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
