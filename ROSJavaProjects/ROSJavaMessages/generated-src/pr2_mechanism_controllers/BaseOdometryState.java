package pr2_mechanism_controllers;

public interface BaseOdometryState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/BaseOdometryState";
  static final java.lang.String _DEFINITION = "geometry_msgs/Twist velocity\nstring[] wheel_link_names\nfloat64[] drive_constraint_errors\nfloat64[] longitudinal_slip_constraint_errors";
  geometry_msgs.Twist getVelocity();
  void setVelocity(geometry_msgs.Twist value);
  java.util.List<java.lang.String> getWheelLinkNames();
  void setWheelLinkNames(java.util.List<java.lang.String> value);
  double[] getDriveConstraintErrors();
  void setDriveConstraintErrors(double[] value);
  double[] getLongitudinalSlipConstraintErrors();
  void setLongitudinalSlipConstraintErrors(double[] value);
}
