package test_roscpp_serialization;

public interface WithMemberNamedHeaderThatIsNotAHeader extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/WithMemberNamedHeaderThatIsNotAHeader";
  static final java.lang.String _DEFINITION = "CustomHeader header\nuint32 a";
  test_roscpp_serialization.CustomHeader getHeader();
  void setHeader(test_roscpp_serialization.CustomHeader value);
  int getA();
  void setA(int value);
}
