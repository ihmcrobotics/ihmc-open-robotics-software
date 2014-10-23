package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\n# This message commands the controller to move a hand to a given position\n#  and orientation. The position/orientation may be specified in world\n# frame, chest frame, or desired joint angles.\n\n#Options for enum\n# uint8 L = 0\n# uint8 R = 1\nuint8 robotSide\n#Options for enum\n# uint8 HAND_POSE = 0\n# uint8 JOINT_ANGLES = 1\nuint8 dataType\n#Options for enum\n# uint8 CHEST = 0\n# uint8 WORLD = 1\nuint8 referenceFrame\n# toHomePosition can be used to move the hands back to their starting position: down and beside the robot\nbool toHomePosition\ngeometry_msgs/Vector3 position\ngeometry_msgs/Quaternion orientation\n# trajectoryTime specifies how fast to move the hand to the desired position\nfloat64 trajectoryTime\n# jointAngles specifies the desired individual arm joint angles. For Atlas, those are:\n# shoulder pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll\nfloat64[] jointAngles\n\n";
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
