package test_rospy;

public interface ArrayVal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/ArrayVal";
  static final java.lang.String _DEFINITION = "Val[] vals\n#Val[10] vals_fixed\n";
  java.util.List<test_rospy.Val> getVals();
  void setVals(java.util.List<test_rospy.Val> value);
}
