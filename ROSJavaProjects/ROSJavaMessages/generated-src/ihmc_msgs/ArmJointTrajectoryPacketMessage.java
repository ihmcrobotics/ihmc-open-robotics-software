package ihmc_msgs;

public interface ArmJointTrajectoryPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmJointTrajectoryPacketMessage";
  static final java.lang.String _DEFINITION = "## ArmJointTrajectoryPacketMessage\n# Packet for executing an arm joint trajectory\n\n# Options for robotSide\n# LEFT = 0 - refers to the LEFT side of a robot\n# RIGHT = 1 - refers to the RIGHT side of a robot\nuint8 robot_side\n\nTrajectoryPointMessage[] trajectory_points\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  java.util.List<ihmc_msgs.TrajectoryPointMessage> getTrajectoryPoints();
  void setTrajectoryPoints(java.util.List<ihmc_msgs.TrajectoryPointMessage> value);
}
