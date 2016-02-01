package test_roscpp_serialization;

public interface ArrayOfVariableLength extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/ArrayOfVariableLength";
  static final java.lang.String _DEFINITION = "VariableLength[4] a";
  java.util.List<test_roscpp_serialization.VariableLength> getA();
  void setA(java.util.List<test_roscpp_serialization.VariableLength> value);
}
