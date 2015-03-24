package ihmc_msgs;

public interface HandPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HandPosePacketMessage";
  static final java.lang.String _DEFINITION = "## HandPosePacketMessage\n# This message commands the controller to move an arm end effector to a given\n# position and orientation. On Atlas, the end effector position is considered\n# as the end of the hand attachment plate, which is about 10 cm from the end\n# of the wrist. The position/orientation may be specified in world frame, chest\n# frame, or desired joint angles.\n\n# Options for robotSide\n# LEFT = 0 - refers to the LEFT side of a robot\n# RIGHT = 1 - refers to the RIGHT side of a robot\nuint8 robot_side\n\n# toHomePosition can be used to move the arm end effectors back to their starting\n# position, defined as down and beside the robot with slightly bent elbows\nbool to_home_position\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n# jointAngles specifies the desired arm joint angles in order. For Atlas the\n# controller assumes joint angles will be given in the follow order: shoulder\n# pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll\nfloat64[] joint_angles\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  boolean getToHomePosition();
  void setToHomePosition(boolean value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  double[] getJointAngles();
  void setJointAngles(double[] value);
}
