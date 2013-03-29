package test_rospy;

public interface MultipleAddTwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/MultipleAddTwoIntsResponse";
  static final java.lang.String _DEFINITION = "int32 ab\nint32 cd";
  int getAb();
  void setAb(int value);
  int getCd();
  void setCd(int value);
}
