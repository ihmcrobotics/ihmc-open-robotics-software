package arm_navigation_msgs;

public interface OrientationConstraint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/OrientationConstraint";
  static final java.lang.String _DEFINITION = "# This message contains the definition of an orientation constraint.\nHeader header\n\n# The robot link this constraint refers to\nstring link_name\n\n# The type of the constraint\nint32 type\nint32 LINK_FRAME=0\nint32 HEADER_FRAME=1\n\n# The desired orientation of the robot link specified as a quaternion\ngeometry_msgs/Quaternion orientation\n\n# optional RPY error tolerances specified if \nfloat64 absolute_roll_tolerance\nfloat64 absolute_pitch_tolerance\nfloat64 absolute_yaw_tolerance\n\n# Constraint weighting factor - a weight for this constraint\nfloat64 weight\n";
  static final int LINK_FRAME = 0;
  static final int HEADER_FRAME = 1;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  int getType();
  void setType(int value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getAbsoluteRollTolerance();
  void setAbsoluteRollTolerance(double value);
  double getAbsolutePitchTolerance();
  void setAbsolutePitchTolerance(double value);
  double getAbsoluteYawTolerance();
  void setAbsoluteYawTolerance(double value);
  double getWeight();
  void setWeight(double value);
}
