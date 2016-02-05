package test_rospy;

public interface StringStringRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/StringStringRequest";
  static final java.lang.String _DEFINITION = "std_msgs/String str\nVal str2\n";
  std_msgs.String getStr();
  void setStr(std_msgs.String value);
  test_rospy.Val getStr2();
  void setStr2(test_rospy.Val value);
}
