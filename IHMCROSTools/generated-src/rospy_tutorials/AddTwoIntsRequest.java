package rospy_tutorials;

public interface AddTwoIntsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rospy_tutorials/AddTwoIntsRequest";
  static final java.lang.String _DEFINITION = "int64 a\nint64 b\n";
  long getA();
  void setA(long value);
  long getB();
  void setB(long value);
}
