package test_rospy;

public interface MultipleAddTwoIntsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/MultipleAddTwoIntsRequest";
  static final java.lang.String _DEFINITION = "# test case for having multiple return values\nint32 a\nint32 b\nint32 c\nint32 d\n";
  int getA();
  void setA(int value);
  int getB();
  void setB(int value);
  int getC();
  void setC(int value);
  int getD();
  void setD(int value);
}
