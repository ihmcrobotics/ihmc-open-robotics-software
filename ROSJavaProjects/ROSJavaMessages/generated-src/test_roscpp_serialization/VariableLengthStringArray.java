package test_roscpp_serialization;

public interface VariableLengthStringArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/VariableLengthStringArray";
  static final java.lang.String _DEFINITION = "string[] foo\n";
  java.util.List<java.lang.String> getFoo();
  void setFoo(java.util.List<java.lang.String> value);
}
