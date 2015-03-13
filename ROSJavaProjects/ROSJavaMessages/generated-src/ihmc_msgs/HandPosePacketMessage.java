package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\r\n# This message commands the controller to move an arm end effector to a given\r\n# position and orientation. On Atlas, the end effector position is considered\r\n# as the end of the hand attachment plate, which is about 10 cm from the end\r\n# of the wrist. The position/orientation may be specified in world frame, chest\r\n# frame, or desired joint angles.\r\n\r\n#Options for robotSide\r\n# uint8 L = 0\r\n# uint8 R = 1\r\nuint8 robotSide\r\n\r\n#Options for dataType\r\n# uint8 HAND_POSE = 0\r\n# uint8 JOINT_ANGLES = 1\r\nuint8 dataType\r\n\r\n#Options for referenceFrame\r\n# uint8 CHEST = 0\r\n# uint8 WORLD = 1\r\nuint8 referenceFrame\r\n\r\n# toHomePosition can be used to move the arm end effectors back to their starting\r\n# position, defined as down and beside the robot with slightly bent elbows\r\nbool toHomePosition\r\n\r\ngeometry_msgs/Vector3 position\r\n\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# trajectoryTime specifies how fast or how slow to move to the desired pose\r\nfloat64 trajectoryTime\r\n\r\n# jointAngles specifies the desired arm joint angles in order. For Atlas the\r\n# controller assumes joint angles will be given in the follow order: shoulder\r\n# pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll\r\nfloat64[] jointAngles\r\n\r\nint8 destination\r\n\r\n\r\n";
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
  byte getDestination();
  void setDestination(byte value);
}
