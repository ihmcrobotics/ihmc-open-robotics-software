package test_rospy;

public interface ListReturnRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/ListReturnRequest";
  static final java.lang.String _DEFINITION = "# test case for having single list return value\nint32 a\nint32 b\nint32 c\nint32 d\n";
  int getA();
  void setA(int value);
  int getB();
  void setB(int value);
  int getC();
  void setC(int value);
  int getD();
  void setD(int value);
}
