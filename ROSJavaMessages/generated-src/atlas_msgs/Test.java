package atlas_msgs;

public interface Test extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/Test";
  static final java.lang.String _DEFINITION = "# test Message\n# Do not use, testing only\nHeader header\n\nfloat32[] damping\nfloat32[] kp_position\nfloat32[] ki_position\nfloat32[] kd_position\nfloat32[] kp_velocity\nfloat32[] i_effort_min\nfloat32[] i_effort_max\nuint8[] k_effort\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getDamping();
  void setDamping(float[] value);
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
  org.jboss.netty.buffer.ChannelBuffer getKEffort();
  void setKEffort(org.jboss.netty.buffer.ChannelBuffer value);
}
