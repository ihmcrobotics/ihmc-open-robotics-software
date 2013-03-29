package test_rospy;

public interface TransitiveMsg1 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/TransitiveMsg1";
  static final java.lang.String _DEFINITION = "TransitiveMsg2 msg2\n\n";
  test_rospy.TransitiveMsg2 getMsg2();
  void setMsg2(test_rospy.TransitiveMsg2 value);
}
