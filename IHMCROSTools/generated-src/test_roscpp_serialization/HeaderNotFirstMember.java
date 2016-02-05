package test_roscpp_serialization;

public interface HeaderNotFirstMember extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/HeaderNotFirstMember";
  static final java.lang.String _DEFINITION = "int8 foo\nHeader header\n";
  byte getFoo();
  void setFoo(byte value);
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
}
