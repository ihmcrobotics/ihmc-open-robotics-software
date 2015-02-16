package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\n# This message commands the controller to move an arm end effector to a given\n# position and orientation. On Atlas, the end effector position is considered\n# as the end of the hand attachment plate, which is about 10 cm from the end\n# of the wrist. The position/orientation may be specified in world frame, chest\n# frame, or desired joint angles.\n\n#Options for robotSide\n# uint8 L = 0\n# uint8 R = 1\nuint8 robotSide\n\n#Options for dataType\n# uint8 HAND_POSE = 0\n# uint8 JOINT_ANGLES = 1\nuint8 dataType\n\n#Options for referenceFrame\n# uint8 CHEST = 0\n# uint8 WORLD = 1\nuint8 referenceFrame\n\n# toHomePosition can be used to move the arm end effectors back to their starting\n# position, defined as down and beside the robot with slightly bent elbows\nbool toHomePosition\n\ngeometry_msgs/Vector3 position\n\ngeometry_msgs/Quaternion orientation\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectoryTime\n\n# jointAngles specifies the desired arm joint angles in order. For Atlas the\n# controller assumes joint angles will be given in the follow order: shoulder\n# pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll\nfloat64[] jointAngles\n\n\n";
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
