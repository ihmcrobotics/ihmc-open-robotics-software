package test_roscpp;

public interface TestWithHeader extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp/TestWithHeader";
  static final java.lang.String _DEFINITION = "Header header\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
}
