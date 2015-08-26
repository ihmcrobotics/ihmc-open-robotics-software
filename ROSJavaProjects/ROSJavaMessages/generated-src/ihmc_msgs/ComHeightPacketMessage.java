package ihmc_msgs;

public interface ComHeightPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ComHeightPacketMessage";
  static final java.lang.String _DEFINITION = "## ComHeightPacketMessage\n# This message sets the robot\'s center of mass height.\n\n# heightOffset specifies CoM height relative to the default starting height, which is\n# about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about\n# 68.9 cm above ground level\nfloat64 height_offset\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.Only the unique id in the top level message is used, the unique id in nested messages is ignored.Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  double getHeightOffset();
  void setHeightOffset(double value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
