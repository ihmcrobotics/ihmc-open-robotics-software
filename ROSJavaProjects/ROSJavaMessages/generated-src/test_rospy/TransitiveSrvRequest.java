package test_rospy;

public interface TransitiveSrvRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/TransitiveSrvRequest";
  static final java.lang.String _DEFINITION = "test_rospy/TransitiveMsg1 msg\n";
  test_rospy.TransitiveMsg1 getMsg();
  void setMsg(test_rospy.TransitiveMsg1 value);
}
