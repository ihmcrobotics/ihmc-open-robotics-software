package test_roscpp_serialization;

public interface CustomHeader extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/CustomHeader";
  static final java.lang.String _DEFINITION = "uint32 a\n";
  int getA();
  void setA(int value);
}
