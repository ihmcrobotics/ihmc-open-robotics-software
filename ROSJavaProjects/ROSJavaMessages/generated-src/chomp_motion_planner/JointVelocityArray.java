package chomp_motion_planner;

public interface JointVelocityArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/JointVelocityArray";
  static final java.lang.String _DEFINITION = "# A list of joint names\nstring[] joint_names\n\n# Velocity for each of the above joints\nfloat64[] velocities\n";
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  double[] getVelocities();
  void setVelocities(double[] value);
}
