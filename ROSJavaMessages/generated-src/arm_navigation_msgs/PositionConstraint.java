package arm_navigation_msgs;

public interface PositionConstraint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/PositionConstraint";
  static final java.lang.String _DEFINITION = "# This message contains the definition of a position constraint.\nHeader header\n\n# The robot link this constraint refers to\nstring link_name\n\n# The offset (in the link frame) for the target point on the link we are planning for\ngeometry_msgs/Point target_point_offset\n\n# The nominal/target position for the point we are planning for\ngeometry_msgs/Point position\n\n# The shape of the bounded region that constrains the position of the end-effector\n# This region is always centered at the position defined above\narm_navigation_msgs/Shape constraint_region_shape\n\n# The orientation of the bounded region that constrains the position of the end-effector. \n# This allows the specification of non-axis aligned constraints\ngeometry_msgs/Quaternion constraint_region_orientation\n\n# Constraint weighting factor - a weight for this constraint\nfloat64 weight\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  geometry_msgs.Point getTargetPointOffset();
  void setTargetPointOffset(geometry_msgs.Point value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  arm_navigation_msgs.Shape getConstraintRegionShape();
  void setConstraintRegionShape(arm_navigation_msgs.Shape value);
  geometry_msgs.Quaternion getConstraintRegionOrientation();
  void setConstraintRegionOrientation(geometry_msgs.Quaternion value);
  double getWeight();
  void setWeight(double value);
}
