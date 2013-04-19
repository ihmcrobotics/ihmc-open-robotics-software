package arm_navigation_msgs;

public interface SimplePoseConstraint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/SimplePoseConstraint";
  static final java.lang.String _DEFINITION = "# This message contains the definition of a simple pose constraint \n# that specifies the pose for a particular link of the robot and corresponding\n# (absolute) position and orientation tolerances\n\n# The standard ROS message header\nHeader header\n\n# The robot link this constraint refers to\nstring link_name\n\n# The desired position of the robot link\ngeometry_msgs/Pose pose\n\n# Position (absolute) tolerance\ngeometry_msgs/Point absolute_position_tolerance\n\n# Orientation (absolute) tolerance\nfloat64 absolute_roll_tolerance\nfloat64 absolute_yaw_tolerance\nfloat64 absolute_pitch_tolerance\n\nint32 orientation_constraint_type\nint32 HEADER_FRAME=0\nint32 LINK_FRAME=1\n";
  static final int HEADER_FRAME = 0;
  static final int LINK_FRAME = 1;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  geometry_msgs.Point getAbsolutePositionTolerance();
  void setAbsolutePositionTolerance(geometry_msgs.Point value);
  double getAbsoluteRollTolerance();
  void setAbsoluteRollTolerance(double value);
  double getAbsoluteYawTolerance();
  void setAbsoluteYawTolerance(double value);
  double getAbsolutePitchTolerance();
  void setAbsolutePitchTolerance(double value);
  int getOrientationConstraintType();
  void setOrientationConstraintType(int value);
}
