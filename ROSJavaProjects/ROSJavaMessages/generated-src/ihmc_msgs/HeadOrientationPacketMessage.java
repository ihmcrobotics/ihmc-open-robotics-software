package ihmc_msgs;

public interface HeadOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HeadOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## HeadOrientationPacketMessage\n# This message gives the desired head orientation of the robot in world coordinates.\n\ngeometry_msgs/Quaternion orientation\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n\n";
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
}
