package test_ros;

public interface AddTwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/AddTwoIntsResponse";
  static final java.lang.String _DEFINITION = "int64 sum";
  long getSum();
  void setSum(long value);
}
