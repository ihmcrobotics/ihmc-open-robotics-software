package test_rosservice;

public interface HeaderEchoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosservice/HeaderEchoResponse";
  static final java.lang.String _DEFINITION = "Header header";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
}
