package test_roscpp;

public interface TestStringInt extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp/TestStringInt";
  static final java.lang.String _DEFINITION = "string str\nint32 counter\n";
  java.lang.String getStr();
  void setStr(java.lang.String value);
  int getCounter();
  void setCounter(int value);
}
