package roscpp_tutorials;

public interface TwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "roscpp_tutorials/TwoIntsResponse";
  static final java.lang.String _DEFINITION = "int64 sum";
  long getSum();
  void setSum(long value);
}
