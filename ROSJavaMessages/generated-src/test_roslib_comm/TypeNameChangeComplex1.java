package test_roslib_comm;

public interface TypeNameChangeComplex1 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/TypeNameChangeComplex1";
  static final java.lang.String _DEFINITION = "SameSubMsg1[] a\nSameSubMsg2[10] b\n";
  java.util.List<test_roslib_comm.SameSubMsg1> getA();
  void setA(java.util.List<test_roslib_comm.SameSubMsg1> value);
  java.util.List<test_roslib_comm.SameSubMsg2> getB();
  void setB(java.util.List<test_roslib_comm.SameSubMsg2> value);
}
