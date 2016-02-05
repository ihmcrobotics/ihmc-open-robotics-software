package roscpp_tutorials;

public interface TwoIntsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "roscpp_tutorials/TwoIntsRequest";
  static final java.lang.String _DEFINITION = "int64 a\nint64 b\n";
  long getA();
  void setA(long value);
  long getB();
  void setB(long value);
}
