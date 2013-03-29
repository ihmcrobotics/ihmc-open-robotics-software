package test_roscpp;

public interface TestArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp/TestArray";
  static final java.lang.String _DEFINITION = "int32 counter\nfloat64[] float_arr\n";
  int getCounter();
  void setCounter(int value);
  double[] getFloatArr();
  void setFloatArr(double[] value);
}
