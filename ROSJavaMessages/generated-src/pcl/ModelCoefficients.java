package pcl;

public interface ModelCoefficients extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pcl/ModelCoefficients";
  static final java.lang.String _DEFINITION = "Header header\nfloat32[] values\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getValues();
  void setValues(float[] value);
}
