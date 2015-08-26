package ihmc_msgs;

public interface FootPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootPosePacketMessage";
  static final java.lang.String _DEFINITION = "## FootPosePacketMessage\n# This message commands the controller to move the desired foot to a given location\n# and orientation in world coordinates. This message is not intended to specify footsteps\n# as the controller will use a straight-line trajectory when moving the foot from its\n# current to desired location/orientation\n\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\ngeometry_msgs/Vector3 position\n\ngeometry_msgs/Quaternion orientation\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectory_time\n\n# A unique id for the current message. This can be a timestamp or sequence number.Only the unique id in the top level message is used, the unique id in nested messages is ignored.Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
