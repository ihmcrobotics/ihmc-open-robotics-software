package atlas_msgs;

public interface AtlasCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasCommand";
  static final java.lang.String _DEFINITION = "# Joint Command Message\n# This structure contains the gains to be applied to a joint.\n# The controller is a PID with feedforward desired torque:\n#\n# This message has been carefully constructed to be less\n# than 1500 in size when serialized, to accommodate transfer\n# UDP.\n#\n#   kp_position     * ( position - measured_position )       +\n#   ki_position     * 1/s * ( position - measured_position ) +\n#   kd_position     * s * ( position - measured_position ) +\n#   kp_velocity    * ( velocity - measured_velocity )     +\n#   effort\n#\nHeader header\n\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\nfloat32[] kp_position\nfloat32[] ki_position\nfloat32[] kd_position\nfloat32[] kp_velocity\n\nfloat32[] i_effort_min\nfloat32[] i_effort_max\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double[] getPosition();
  void setPosition(double[] value);
  double[] getVelocity();
  void setVelocity(double[] value);
  double[] getEffort();
  void setEffort(double[] value);
  float[] getKpPosition();
  void setKpPosition(float[] value);
  float[] getKiPosition();
  void setKiPosition(float[] value);
  float[] getKdPosition();
  void setKdPosition(float[] value);
  float[] getKpVelocity();
  void setKpVelocity(float[] value);
  float[] getIEffortMin();
  void setIEffortMin(float[] value);
  float[] getIEffortMax();
  void setIEffortMax(float[] value);
}
