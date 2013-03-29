package test_roscpp_serialization;

public interface EmbeddedFixedLength extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/EmbeddedFixedLength";
  static final java.lang.String _DEFINITION = "FixedLength a\n";
  test_roscpp_serialization.FixedLength getA();
  void setA(test_roscpp_serialization.FixedLength value);
}
