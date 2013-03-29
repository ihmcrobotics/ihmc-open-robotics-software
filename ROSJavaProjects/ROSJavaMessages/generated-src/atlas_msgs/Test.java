package atlas_msgs;

public interface Test extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/Test";
  static final java.lang.String _DEFINITION = "# test Message\n# Do not use, testing only\nHeader header\n\nstring[] name\nuint32[] index\nfloat64[] damping\nfloat64[] kp_position\nfloat64[] ki_position\nfloat64[] kd_position\nfloat64[] kp_velocity\nfloat64[] i_effort_min\nfloat64[] i_effort_max\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  int[] getIndex();
  void setIndex(int[] value);
  double[] getDamping();
  void setDamping(double[] value);
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
