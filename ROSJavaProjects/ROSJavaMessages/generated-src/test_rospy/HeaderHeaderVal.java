package test_rospy;

public interface HeaderHeaderVal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/HeaderHeaderVal";
  static final java.lang.String _DEFINITION = "Header header\nHeaderVal val";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  test_rospy.HeaderVal getVal();
  void setVal(test_rospy.HeaderVal value);
}
