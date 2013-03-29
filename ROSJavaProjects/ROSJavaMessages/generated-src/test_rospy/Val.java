package test_rospy;

public interface Val extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/Val";
  static final java.lang.String _DEFINITION = "string val";
  java.lang.String getVal();
  void setVal(java.lang.String value);
}
