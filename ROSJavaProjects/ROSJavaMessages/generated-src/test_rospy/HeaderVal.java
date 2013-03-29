package test_rospy;

public interface HeaderVal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/HeaderVal";
  static final java.lang.String _DEFINITION = "Header header\nstring val";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getVal();
  void setVal(java.lang.String value);
}
