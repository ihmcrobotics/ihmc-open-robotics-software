package osrf_msgs;

public interface JointCommands extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "osrf_msgs/JointCommands";
  static final java.lang.String _DEFINITION = "# Joint Command Message\n# This structure contains the gains to be applied to a joint.\n# The controller is a PID with feedforward desired torque:\n#\n#   kp_position     * ( position - measured_position )       +\n#   ki_position     * 1/s * ( position - measured_position ) +\n#   kd_position     * s * ( position - measured_position ) +\n#   kp_velocity    * ( velocity - measured_velocity )     +\n#   effort\n#\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\nfloat64[] kp_position\nfloat64[] ki_position\nfloat64[] kd_position\nfloat64[] kp_velocity\n\nfloat64[] i_effort_min\nfloat64[] i_effort_max\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  double[] getPosition();
  void setPosition(double[] value);
  double[] getVelocity();
  void setVelocity(double[] value);
  double[] getEffort();
  void setEffort(double[] value);
  double[] getKpPosition();
  void setKpPosition(double[] value);
  double[] getKiPosition();
  void setKiPosition(double[] value);
  double[] getKdPosition();
  void setKdPosition(double[] value);
  double[] getKpVelocity();
  void setKpVelocity(double[] value);
  double[] getIEffortMin();
  void setIEffortMin(double[] value);
  double[] getIEffortMax();
  void setIEffortMax(double[] value);
}
