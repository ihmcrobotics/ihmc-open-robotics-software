package test_roscpp_serialization;

public interface ArrayOfFixedLength extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/ArrayOfFixedLength";
  static final java.lang.String _DEFINITION = "# This comment has \"quotes\" in it\nFixedLength[4] a";
  java.util.List<test_roscpp_serialization.FixedLength> getA();
  void setA(java.util.List<test_roscpp_serialization.FixedLength> value);
}
