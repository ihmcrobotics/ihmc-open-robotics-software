package test_rospy;

public interface EmbedTest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/EmbedTest";
  static final java.lang.String _DEFINITION = "std_msgs/String str1\nstd_msgs/Int32 int1\nstd_msgs/Int32[] ints\nVal val\nVal[] vals\nArrayVal[] arrayval\n";
  std_msgs.String getStr1();
  void setStr1(std_msgs.String value);
  std_msgs.Int32 getInt1();
  void setInt1(std_msgs.Int32 value);
  java.util.List<std_msgs.Int32> getInts();
  void setInts(java.util.List<std_msgs.Int32> value);
  test_rospy.Val getVal();
  void setVal(test_rospy.Val value);
  java.util.List<test_rospy.Val> getVals();
  void setVals(java.util.List<test_rospy.Val> value);
  java.util.List<test_rospy.ArrayVal> getArrayval();
  void setArrayval(java.util.List<test_rospy.ArrayVal> value);
}
