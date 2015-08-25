package ihmc_msgs;

public interface ComHeightPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ComHeightPacketMessage";
  static final java.lang.String _DEFINITION = "## ComHeightPacketMessage\n# This message sets the robot\'s center of mass height.\n\n# heightOffset specifies CoM height relative to the default starting height, which is\n# about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about\n# 68.9 cm above ground level\nfloat64 height_offset\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n\n";
  double getHeightOffset();
  void setHeightOffset(double value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
}
