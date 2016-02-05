package test_ros;

public interface TestHeader extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/TestHeader";
  static final java.lang.String _DEFINITION = "Header header\n\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nbyte auto_header # autoset header on response\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getCallerId();
  void setCallerId(java.lang.String value);
  java.lang.String getOrigCallerId();
  void setOrigCallerId(java.lang.String value);
  byte getAutoHeader();
  void setAutoHeader(byte value);
}
