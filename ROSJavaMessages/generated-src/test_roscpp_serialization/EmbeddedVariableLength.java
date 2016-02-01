package test_roscpp_serialization;

public interface EmbeddedVariableLength extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/EmbeddedVariableLength";
  static final java.lang.String _DEFINITION = "VariableLength a\n";
  test_roscpp_serialization.VariableLength getA();
  void setA(test_roscpp_serialization.VariableLength value);
}
