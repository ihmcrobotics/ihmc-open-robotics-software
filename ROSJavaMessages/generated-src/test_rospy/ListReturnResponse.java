package test_rospy;

public interface ListReturnResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/ListReturnResponse";
  static final java.lang.String _DEFINITION = "int32[] abcd";
  int[] getAbcd();
  void setAbcd(int[] value);
}
