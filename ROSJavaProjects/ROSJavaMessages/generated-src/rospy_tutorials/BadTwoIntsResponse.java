package rospy_tutorials;

public interface BadTwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rospy_tutorials/BadTwoIntsResponse";
  static final java.lang.String _DEFINITION = "int32 sum";
  int getSum();
  void setSum(int value);
}
