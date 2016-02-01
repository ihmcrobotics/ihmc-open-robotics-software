package test_roslib_comm;

public interface SameSubMsg1 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/SameSubMsg1";
  static final java.lang.String _DEFINITION = "int32 a\nfloat32 b\nstring c\nuint64[10] d\nfloat64[] e";
  int getA();
  void setA(int value);
  float getB();
  void setB(float value);
  java.lang.String getC();
  void setC(java.lang.String value);
  long[] getD();
  void setD(long[] value);
  double[] getE();
  void setE(double[] value);
}
