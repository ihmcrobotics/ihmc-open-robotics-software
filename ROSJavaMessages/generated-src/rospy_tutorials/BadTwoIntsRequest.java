package rospy_tutorials;

public interface BadTwoIntsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rospy_tutorials/BadTwoIntsRequest";
  static final java.lang.String _DEFINITION = "# Bad version of AddTwoInts for unit testing\nint64 a\nint32 b\n";
  long getA();
  void setA(long value);
  int getB();
  void setB(int value);
}
