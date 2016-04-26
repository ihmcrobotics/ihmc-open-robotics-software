package ihmc_msgs;

public interface SO3TrajectoryPointRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SO3TrajectoryPointRosMessage";
  static final java.lang.String _DEFINITION = "## SO3TrajectoryPointRosMessage\n# This class is used to build trajectory messages in taskspace. It holds the only the rotational information for one trajectory point (orientation & angular velocity). Feel free to look at EuclideanTrajectoryPointMessage (translational) and SE3TrajectoryPointMessage (rotational AND translational)\n\n# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.\nfloat64 time\n\n# Define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.\ngeometry_msgs/Quaternion orientation\n\n# Define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.\ngeometry_msgs/Vector3 angular_velocity\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  double getTime();
  void setTime(double value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  long getUniqueId();
  void setUniqueId(long value);
}
