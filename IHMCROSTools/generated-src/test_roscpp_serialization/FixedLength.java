package test_roscpp_serialization;

public interface FixedLength extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/FixedLength";
  static final java.lang.String _DEFINITION = "uint32 a\nfloat32 b\n";
  int getA();
  void setA(int value);
  float getB();
  void setB(float value);
}
