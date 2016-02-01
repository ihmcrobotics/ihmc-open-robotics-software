package test_roscpp_serialization;

public interface WithHeader extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/WithHeader";
  static final java.lang.String _DEFINITION = "Header header\nuint32 a\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getA();
  void setA(int value);
}
