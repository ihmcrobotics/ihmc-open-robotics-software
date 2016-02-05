package object_manipulation_msgs;

public interface CartesianGains extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/CartesianGains";
  static final java.lang.String _DEFINITION = "Header header\n\nfloat64[] gains\nfloat64[] fixed_frame\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double[] getGains();
  void setGains(double[] value);
  double[] getFixedFrame();
  void setFixedFrame(double[] value);
}
