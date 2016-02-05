package test_roscpp_serialization;

public interface FixedLengthStringArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/FixedLengthStringArray";
  static final java.lang.String _DEFINITION = "string[5] foo\n";
  java.util.List<java.lang.String> getFoo();
  void setFoo(java.util.List<java.lang.String> value);
}
