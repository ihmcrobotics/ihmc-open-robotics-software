package ihmc_msgs;

public interface FootPosePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootPosePacketMessage";
  static final java.lang.String _DEFINITION = "## FootPosePacketMessage\r\n# This message commands the controller to move the desired foot to a given location\r\n# and orientation in world coordinates. This message is not intended to specify footsteps\r\n# as the controller will use a straight-line trajectory when moving the foot from its\r\n# current to desired location/orientation\r\n\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\ngeometry_msgs/Vector3 position\r\n\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# trajectoryTime specifies how fast or how slow to move to the desired pose\r\nfloat64 trajectory_time\r\n\r\n\r\n";
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
}
