package test_roslib_comm;

public interface HeaderTest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/HeaderTest";
  static final java.lang.String _DEFINITION = "Header header\nint32 i32";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getI32();
  void setI32(int value);
}
