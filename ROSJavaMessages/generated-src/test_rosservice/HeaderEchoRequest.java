package test_rosservice;

public interface HeaderEchoRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosservice/HeaderEchoRequest";
  static final java.lang.String _DEFINITION = "Header header\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
}
