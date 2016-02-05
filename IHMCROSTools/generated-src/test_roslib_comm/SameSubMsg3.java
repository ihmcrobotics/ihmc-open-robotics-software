package test_roslib_comm;

public interface SameSubMsg3 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/SameSubMsg3";
  static final java.lang.String _DEFINITION = "\n# Lots of comments\nint32 a  #And more comments\nfloat32 b\nstring c\n# And also some white space\n\n\nuint64[10] d\n float64[] e\n\n";
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
