package rospy_tutorials;

public interface HeaderString extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rospy_tutorials/HeaderString";
  static final java.lang.String _DEFINITION = "Header header\nstring data\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getData();
  void setData(java.lang.String value);
}
