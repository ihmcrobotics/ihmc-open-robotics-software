package atlas_msgs;

public interface AtlasCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasCommand";
  static final java.lang.String _DEFINITION = "# Joint Command Message\n# This structure contains the gains to be applied to a joint.\n# The controller is a PID with feedforward desired torque:\n#\n# This message has been carefully constructed to be less\n# than 1500 in size when serialized, to accommodate transfer\n# UDP.\n#\n#   k_effort * (\n#     kp_position     * ( position - measured_position )       +\n#     ki_position     * 1/s * ( position - measured_position ) +\n#     kd_position     * s * ( position - measured_position ) +\n#     kp_velocity     * ( velocity - measured_velocity )     +\n#     effort ) +\n#   (1 - k_effort)  * effort_bdi\n#\n#  Where effort_bdi comes from AtlasSimInterface Dynamics Behavior Library.\n#\nHeader header\n\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\nfloat32[] kp_position\nfloat32[] ki_position\nfloat32[] kd_position\nfloat32[] kp_velocity\nuint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, \n                       # at run time, a double between 0 and 1 is obtained\n                       # by dividing by 255.0d.\n\nfloat32[] i_effort_min\nfloat32[] i_effort_max\n\nuint8 desired_controller_period_ms # max allowed controller update period in milli-seconds simulation time for controller synchronization.  See SynchronizationStatistics.msg for tracking synchronization status.\n";
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
  org.jboss.netty.buffer.ChannelBuffer getKEffort();
  void setKEffort(org.jboss.netty.buffer.ChannelBuffer value);
  float[] getIEffortMin();
  void setIEffortMin(float[] value);
  float[] getIEffortMax();
  void setIEffortMax(float[] value);
  byte getDesiredControllerPeriodMs();
  void setDesiredControllerPeriodMs(byte value);
}
