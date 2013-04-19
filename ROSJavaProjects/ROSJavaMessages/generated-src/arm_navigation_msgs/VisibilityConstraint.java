package arm_navigation_msgs;

public interface VisibilityConstraint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/VisibilityConstraint";
  static final java.lang.String _DEFINITION = "# This message contains the definition of a visibility constraint.\nHeader header\n\n# The point stamped target that needs to be kept within view of the sensor\ngeometry_msgs/PointStamped target\n\n# The local pose of the frame in which visibility is to be maintained\n# The frame id should represent the robot link to which the sensor is attached\n# The visual axis of the sensor is assumed to be along the X axis of this frame\ngeometry_msgs/PoseStamped sensor_pose\n\n# The deviation (in radians) that will be tolerated\n# Constraint error will be measured as the solid angle between the \n# X axis of the frame defined above and the vector between the origin \n# of the frame defined above and the target location\nfloat64 absolute_tolerance\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.PointStamped getTarget();
  void setTarget(geometry_msgs.PointStamped value);
  geometry_msgs.PoseStamped getSensorPose();
  void setSensorPose(geometry_msgs.PoseStamped value);
  double getAbsoluteTolerance();
  void setAbsoluteTolerance(double value);
}
