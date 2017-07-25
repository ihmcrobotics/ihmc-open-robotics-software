package ihmc_msgs;

public interface SE3TrajectoryPointRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SE3TrajectoryPointRosMessage";
  static final java.lang.String _DEFINITION = "## SE3TrajectoryPointRosMessage\n# This class is used to build trajectory messages in taskspace. It holds the necessary information for\n# one trajectory point. Feel free to look at EuclideanTrajectoryPointMessage (translational) and\n# EuclideanTrajectoryPointMessage (rotational)\n\n# Time at which the trajectory point has to be reached. The time is relative to when the trajectory\n# starts.\nfloat64 time\n\n# Define the desired 3D position to be reached at this trajectory point.\ngeometry_msgs/Point position\n\n# Define the desired 3D orientation to be reached at this trajectory point.\ngeometry_msgs/Quaternion orientation\n\n# Define the desired 3D linear velocity to be reached at this trajectory point.\ngeometry_msgs/Vector3 linear_velocity\n\n# Define the desired 3D angular velocity to be reached at this trajectory point.\ngeometry_msgs/Vector3 angular_velocity\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  double getTime();
  void setTime(double value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getLinearVelocity();
  void setLinearVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  long getUniqueId();
  void setUniqueId(long value);
}
