package ihmc_msgs;

public interface HeadOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HeadOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## HeadOrientationPacketMessage\n# This message gives the desired head orientation of the robot in world coordinates.\n\ngeometry_msgs/Quaternion orientation\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
