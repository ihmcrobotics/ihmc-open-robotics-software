package pr2_mechanism_controllers;

public interface BaseControllerState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/BaseControllerState";
  static final java.lang.String _DEFINITION = "geometry_msgs/Twist command\nfloat64[] joint_velocity_measured\nfloat64[] joint_velocity_commanded\nfloat64[] joint_velocity_error\nfloat64[] joint_effort_measured\nfloat64[] joint_effort_commanded\nfloat64[] joint_effort_error\nstring[] joint_names\n\n";
  geometry_msgs.Twist getCommand();
  void setCommand(geometry_msgs.Twist value);
  double[] getJointVelocityMeasured();
  void setJointVelocityMeasured(double[] value);
  double[] getJointVelocityCommanded();
  void setJointVelocityCommanded(double[] value);
  double[] getJointVelocityError();
  void setJointVelocityError(double[] value);
  double[] getJointEffortMeasured();
  void setJointEffortMeasured(double[] value);
  double[] getJointEffortCommanded();
  void setJointEffortCommanded(double[] value);
  double[] getJointEffortError();
  void setJointEffortError(double[] value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
}
