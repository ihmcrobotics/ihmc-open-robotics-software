package test_rospy;

public interface TransitiveImport extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/TransitiveImport";
  static final java.lang.String _DEFINITION = "# Bug #2133/2139: EmbedTest uses std_msgs, so TransitiveImport needs it as well\nEmbedTest data";
  test_rospy.EmbedTest getData();
  void setData(test_rospy.EmbedTest value);
}
