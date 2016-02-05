package test_roscpp_serialization;

public interface VariableLengthArrayOfExternal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/VariableLengthArrayOfExternal";
  static final java.lang.String _DEFINITION = "# This comment has \"quotes\" in it\nrosgraph_msgs/Log[] a";
  java.util.List<rosgraph_msgs.Log> getA();
  void setA(java.util.List<rosgraph_msgs.Log> value);
}
